import rclpy
from rclpy.node import Node
import threading
import nats
import asyncio

from msgs.msg import Float32Stamped
from std_msgs.msg import Int16
from geometry_msgs.msg import WrenchStamped
from manual.utils.keyboard import KeyboardState

MAX_FORCE = 1.0
MAX_TORQUE = 1.0


class KeyboardNode(Node):
    def __init__(self):
        super().__init__("keyboard_node")

        self.wrench_publisher = self.create_publisher(WrenchStamped, "wrench", 10)
        self.light_publisher = self.create_publisher(Int16, "light", 10)
        self.torpedo_publisher = self.create_publisher(Int16, "torpedo", 10)
        self.dropper_publisher = self.create_publisher(Int16, "dropper", 10)
        self.get_logger().info("Keyboard node initialized")

        self.force = 0.5
        self.torque = 0.5

        # Start the async server in a separate thread
        self.async_thread = threading.Thread(target=self._run_async_server, daemon=True)
        self.async_thread.start()
        self.running = True

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

    async def run_server(self):
        self.nc = await nats.connect("nats://localhost:4222")
        try:
            self.get_logger().info("Connected to NATS server")

            sub = await self.nc.subscribe("keyboard")
            while self.running:
                try:
                    msg = await sub.next_msg(timeout=0.5)
                    state = KeyboardState.from_json(msg.data.decode("utf-8"))
                    await self.set_state(state)
                except nats.errors.TimeoutError:
                    self.get_logger().info("No keyboard message received, waiting...")
                    await self.set_state(KeyboardState())
                    continue
                except Exception as e:
                    self.get_logger().error(f"Error processing message: {str(e)}")
                    continue
        except Exception as e:
            self.get_logger().error(f"Error in NATS server: {str(e)}")
            if self.running:
                # Attempt to reconnect after a delay
                await asyncio.sleep(5)
                await self.run_server()
        finally:
            await self.nc.close()

    def destroy_node(self):
        """Clean up resources when the node is destroyed"""
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()

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
