import asyncio
import yaml
from enum import Enum
from gz.transport13 import Node, Publisher, _Node, SubscribeOptions, AdvertiseMessageOptions
import importlib
import json
import base64
from typing import Callable, Awaitable, Dict
from dataclasses import dataclass
import os

DEFAULT_CONFIG_FILE = os.path.join(os.path.dirname(__file__), "gazebo_bridge.yaml")

# Enumeration for message direction
class Direction(Enum):
    LOCAL_TO_DOCKER = "GZ_TO_ROS"
    DOCKER_TO_LOCAL = "ROS_TO_GZ"
    BIDIRECTIONAL = "BIDIRECTIONAL"

@dataclass
class Websocket:
    send: Callable[[str], Awaitable[None]]
    recv: Callable[[], Awaitable[str]]

@dataclass
class PublisherTopic:
    name: str
    publisher: Publisher
    type: str
    bidirectional: bool

class Bridge:
    def __init__(self, out_direction: Direction, in_direction: Direction, ws: Websocket, config_file=DEFAULT_CONFIG_FILE):
        self.node = Node()
        self.queue = asyncio.Queue()
        self.loop = asyncio.get_event_loop()
        self.publisher_topics: Dict[str, PublisherTopic] = {}
        self.ws = ws
        self.out_direction = out_direction
        self.in_direction = in_direction
        
        # Load configuration
        self.config = self.load_config(config_file)
        
        # Set up subscriptions based on config
        self.setup_subscriptions()

        self.bidirectional_messages = set()

    def load_config(self, config_file):
        try:
            with open(config_file, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            print(f"Error loading config file: {e}")
            return []

    def setup_subscriptions(self):
        for topic_config in self.config:
            gz_topic = topic_config["gz_topic_name"]
            gz_type = topic_config["gz_type_name"]
            direction = topic_config["direction"]
            
            # Parse the Gazebo message type
            msg_module_path, msg_class = self.parse_msg_type(gz_type)
            bidirectional = direction == Direction.BIDIRECTIONAL.value
            
            if direction in [self.out_direction.value, Direction.BIDIRECTIONAL.value]:
                print(f"Setting up subscription for {gz_topic} with type {gz_type}")
                
                # Create callback for this specific topic
                callback = self.create_callback(gz_topic, bidirectional)
                
                # Subscribe to the topic
                self.node.subscribe_raw(
                    gz_topic, callback, msg_class.DESCRIPTOR.full_name, SubscribeOptions()
                )

            if direction in [self.in_direction.value, Direction.BIDIRECTIONAL.value]:
                print(f"Setting up publisher for {gz_topic} with type {gz_type}")
                
                # Create a publisher for this topic
                self.publisher_topics[gz_topic] = PublisherTopic(
                    name=gz_topic,
                    publisher=Publisher(
                        _Node.advertise(self.node, gz_topic, msg_class.DESCRIPTOR.full_name, AdvertiseMessageOptions())
                    ),
                    type=gz_type,
                    bidirectional=bidirectional
                )

    def parse_msg_type(self, gz_type):
        # Expected format: "gz.msgs.TypeName"
        parts = gz_type.split('.')
        
        if len(parts) < 3:
            raise ValueError(f"Invalid message type format: {gz_type}")
        
        # Convert to actual import path (e.g., "gz.msgs10.type_name_pb2")
        module_name = f"{parts[0]}.msgs10.{parts[2].lower()}_pb2"
        class_name = parts[2]
        
        try:
            module = importlib.import_module(module_name)
            return module_name, getattr(module, class_name)
        except (ImportError, AttributeError) as e:
            print(f"Error importing message type {gz_type}: {e}")
            raise

    def create_callback(self, topic, bidirectional):
        print(f"Creating callback for topic: {topic}")
        async def callback(proto_msg, _):
            # Use run_coroutine_threadsafe since callback runs in a different thread
            data = base64.b64encode(proto_msg).decode('utf-8')
            if bidirectional and data in self.bidirectional_messages:
                self.bidirectional_messages.remove(data)
                return
            message_data = {
                "topic": topic,
                "data": data
            }
            await self.queue.put(message_data)
        return lambda proto_msg, _: asyncio.run_coroutine_threadsafe(callback(proto_msg, _), self.loop)

    async def process_incoming_message(self, message):
        # Handle messages coming from Docker
        topic = message["topic"]
        data = message["data"]
        publisher_topic = self.publisher_topics[topic]
        if publisher_topic.bidirectional:
            self.bidirectional_messages.add(data)
        publisher_topic.publisher.publish_raw(base64.b64decode(data), publisher_topic.type)

    async def run(self):
        await asyncio.gather(self.send_messages(), self.receive_messages())
    
    async def send_messages(self):
        while True:
            message = await self.queue.get()
            await self.ws.send(json.dumps(message))
    
    async def receive_messages(self):
        while True:
            data = await self.ws.recv()
            # Handle binary data
            if isinstance(data, bytes):
                print(f"Received binary message from Docker (length: {len(data)} bytes)")
                continue
            try:
                message = json.loads(data)
                await self.process_incoming_message(message)
            except json.JSONDecodeError:
                print(f"Invalid JSON message: {data}")
