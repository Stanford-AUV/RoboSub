import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from manual.utils.joystick import JoystickState
import threading
import asyncio
import nats
from msgs.msg import Float32Stamped
from std_msgs.msg import Int16


class JoystickNode(Node):
    def __init__(self):
        super().__init__("joystick_node")

        self.wrench_publisher = self.create_publisher(WrenchStamped, "wrench", 10)
        self.light_publisher = self.create_publisher(Int16, "light", 10)
        self.torpedo_publisher = self.create_publisher(Int16, "torpedo", 10)
        self.dropper_publisher = self.create_publisher(Int16, "dropper", 10)
        self.get_logger().info("Joystick node initialized")

        ############################################################
        #           TODO: Any additional definitions               #
        ############################################################

        # Start the async server in a separate thread
        self.async_thread = threading.Thread(target=self._run_async_server, daemon=True)
        self.async_thread.start()

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

    async def set_state(self, state: JoystickState):
        pass
        ############################################################
        #           TODO: Process joystick state                   #
        ############################################################


    async def run_server(self):
        self.nc = await nats.connect("nats://localhost:4222")
        try:
            self.get_logger().info("Connected to NATS server")

            sub = await self.nc.subscribe("joystick")
            while self.running:
                try:
                    msg = await sub.next_msg(timeout=0.5)
                    state = JoystickState.from_json(msg.data.decode("utf-8"))
                    await self.set_state(state)
                except nats.errors.TimeoutError:
                    self.get_logger().info("No joystick message received, waiting...")
                    await self.set_state(JoystickState())
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
    node = JoystickNode()

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
