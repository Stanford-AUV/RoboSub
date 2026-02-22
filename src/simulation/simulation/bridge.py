import asyncio
import yaml
from enum import Enum
from gz.transport13 import (
    Node,
    Publisher,
    _Node,
    SubscribeOptions,
    AdvertiseMessageOptions,
)
import importlib
import json
import base64
from typing import Callable, Awaitable, Dict
from dataclasses import dataclass
import os
import nats
from nats.aio.msg import Msg
import time

DEFAULT_CONFIG_FILE = os.path.join(os.path.dirname(__file__), "gazebo_bridge.yaml")


# Enumeration for message direction
class Direction(Enum):
    LOCAL_TO_DOCKER = "GZ_TO_ROS"
    DOCKER_TO_LOCAL = "ROS_TO_GZ"


@dataclass
class PublisherTopic:
    name: str
    publisher: Publisher
    type: str


class Bridge:
    def __init__(
        self,
        out_direction: Direction,
        in_direction: Direction,
        config_file=DEFAULT_CONFIG_FILE,
    ):
        self.node = Node()
        self.queue = asyncio.Queue()
        self.loop = asyncio.get_event_loop()
        self.publisher_topics: Dict[str, PublisherTopic] = {}
        self.out_direction = out_direction
        self.in_direction = in_direction

        # Load configuration
        self.config = self.load_config(config_file)

        # Set up subscriptions based on config
        self.setup_subscriptions()

    def load_config(self, config_file):
        try:
            with open(config_file, "r") as file:
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

            if direction in [self.out_direction.value]:
                print(f"Setting up subscription for {gz_topic} with type {gz_type}")

                # Create callback for this specific topic
                callback = self.create_callback(gz_topic)

                # Subscribe to the topic
                self.node.subscribe_raw(
                    gz_topic,
                    callback,
                    msg_class.DESCRIPTOR.full_name,
                    SubscribeOptions(),
                )

            if direction in [self.in_direction.value]:
                print(f"Setting up publisher for {gz_topic} with type {gz_type}")

                # Create a publisher for this topic
                self.publisher_topics[gz_topic] = PublisherTopic(
                    name=gz_topic,
                    publisher=Publisher(
                        _Node.advertise(
                            self.node,
                            gz_topic,
                            msg_class.DESCRIPTOR.full_name,
                            AdvertiseMessageOptions(),
                        )
                    ),
                    type=gz_type,
                )

    def parse_msg_type(self, gz_type):
        # Expected format: "gz.msgs.TypeName"
        parts = gz_type.split(".")

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

    def create_callback(self, topic):
        print(f"Creating callback for topic: {topic}")

        def raw_callback(proto_msg, _):
            data = base64.b64encode(proto_msg)
            self.loop.call_soon_threadsafe(
                self.queue.put_nowait, {"topic": topic, "data": data}
            )

        return raw_callback

    async def on_message(self, msg):
        # Handle messages coming from Docker
        topic = msg.subject
        data = msg.data
        if topic not in self.publisher_topics:
            return
        publisher_topic = self.publisher_topics[topic]
        publisher_topic.publisher.publish_raw(
            base64.b64decode(data), publisher_topic.type
        )
        time_ = msg.headers["time"]
        # print(" IN", time_, topic)

    async def run(self):
        self.nc = await nats.connect("nats://localhost:4222", no_echo=True)
        sender_task = asyncio.create_task(self.send_messages())
        receiver_task = asyncio.create_task(self.receive_messages())
        await asyncio.gather(sender_task, receiver_task)

    async def receive_messages(self):
        sub = await self.nc.subscribe("*")
        async for msg in sub.messages:
            await self.on_message(msg)

    async def send_messages(self):
        while True:
            message = await self.queue.get()
            try:
                time_ = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                # print("OUT", time_, message["topic"])
                await self.nc.publish(
                    message["topic"], message["data"], headers=dict(time=time_)
                )
            except Exception as e:
                print(f"Error publishing message: {e}")
