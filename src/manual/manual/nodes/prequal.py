import rclpy
from rclpy.node import Node
import threading
import os
import asyncio
import yaml

from msgs.msg import Float32Stamped
from std_msgs.msg import Int16
from geometry_msgs.msg import WrenchStamped
from manual.utils.keyboard import KeyboardState
from manual.keyboard_local import isKeyPressed, keysToState

MAX_FORCE = 1.0
MAX_TORQUE = 1.0


class PrequalNode(Node):
    def __init__(self, path):
        super().__init__("keyboard_node")

        self.wrench_publisher = self.create_publisher(WrenchStamped, "wrench", 10)
        self.light_publisher = self.create_publisher(Int16, "light", 10)
        self.get_logger().info("Keyboard node initialized")

        self.force = 0.5
        self.torque = 0.5

        # Start the async server in a separate thread
        self.async_thread = threading.Thread(target=self._run_async_server, daemon=True)
        self.async_thread.start()
        self.running = True
        self.path = path
        self.start_time = self.get_clock().now()

    def _run_async_server(self):
        """Run the async server in a separate thread"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self.run_server())
        except Exception as e:
            self.get_logger().error(f"Error in async server: {str(e)}")
        finally:
            loop.close()

    async def set_state(self, state: KeyboardState):
        wrench_stamped = WrenchStamped()
        wrench = wrench_stamped.wrench

        x = y = z = th = 0
        if state.moveBackward:
            x = -1
        elif state.moveForward:
            x = 1
        if state.moveLeft:
            y = -1
        elif state.moveRight:
            y = 1
        if state.moveDown:
            z = -1
        elif state.moveUp:
            z = 1
        if state.turnCW:
            th = -1
        elif state.turnCCW:
            th = 1

        if state.decreaseLinVel or state.decreaseVel:
            self.force *= 0.9
        elif state.increaseLinVel or state.increaseVel:
            self.force = min(1.1 * self.force, MAX_FORCE)

        if state.decreaseAngVel or state.decreaseVel:
            self.torque *= 0.9
        elif state.increaseAngVel or state.increaseVel:
            self.torque = min(1.1 * self.torque, MAX_TORQUE)

        wrench.force.x = x * self.force
        wrench.force.y = y * self.force
        wrench.force.z = z * self.force
        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = th * self.torque

        self.wrench_publisher.publish(wrench_stamped)

        lights = Int16()
        # 1100 is OFF, 1900 is ON, 1500 is HALF please fix this
        lights.data = 1100 + int(800 * state.moveForward)

        self.light_publisher.publish(lights)

    async def run_server(self):
        data = self.load_path_yaml()
        for idx, value in data.items():
            print(idx, value)
            self.start_time = self.get_clock().now()
            movement, time, force = value
            state = KeyboardState()
            for key in keysToState:
                state.setState(keysToState[key], 0)
                if isKeyPressed(key, movement):
                    state.setState(keysToState[key], 1)
            self.force = force
            while self.get_clock().now() < self.start_time + time:
                self.set_state(state)

    def destroy_node(self):
        """Clean up resources when the node is destroyed"""
        self.running = False
        super().destroy_node()

    def load_path_yaml(self):
        if not os.path.exists(self.path):
            raise FileNotFoundError("Noooooo! No yaml path exists :(")

        with open(self.path, "r") as f:
            data = yaml.safe_load(f)

        return data


def main(args=None):
    rclpy.init(args=args)

    SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
    yaml_path = os.path.join(SCRIPT_DIR, "..", "prequalPresses.yaml")
    yaml_path = os.path.abspath(yaml_path)

    node = PrequalNode(yaml_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Shutting down...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
