import rclpy
from rclpy.node import Node
from manual.utils.joystick import JoystickState
import threading
import asyncio
import nats
from nav_msgs.msg import Odometry


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        self.waypoint_publisher = self.create_publisher(Odometry, 'waypoint', 10)
        self.get_logger().info('Joystick node initialized')

        # Flag to control the async server
        self.running = True
        
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
            self.get_logger().error(f'Error in async server: {str(e)}')
        finally:
            loop.close()

    async def set_state(self, state: JoystickState):
        waypoint_msg = Odometry()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = "base_link"

        if state.enabled:
            waypoint_msg.twist.twist.linear.x = state.ly * 0.05
            waypoint_msg.twist.twist.linear.y = state.lx * 0.05
            waypoint_msg.twist.twist.linear.z = state.ry * 0.05
            waypoint_msg.twist.twist.angular.x = 0.0
            waypoint_msg.twist.twist.angular.y = 0.0
            waypoint_msg.twist.twist.angular.z = state.rx * 0.05
        else:
            waypoint_msg.twist.twist.linear.x = 0.0
            waypoint_msg.twist.twist.linear.y = 0.0
            waypoint_msg.twist.twist.linear.z = 0.0
            waypoint_msg.twist.twist.angular.x = 0.0
            waypoint_msg.twist.twist.angular.y = 0.0
            waypoint_msg.twist.twist.angular.z = 0.0

        self.get_logger().info(f"Publishing twist: {waypoint_msg.twist.twist}")

        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self.publish_waypoint, waypoint_msg)

    def publish_waypoint(self, msg):
        self.waypoint_publisher.publish(msg)

    async def run_server(self):
        self.nc = await nats.connect("nats://localhost:4222")
        try:
            self.get_logger().info('Connected to NATS server')

            sub = await self.nc.subscribe("joystick")
            while self.running:
                try:
                    msg = await sub.next_msg(timeout=1.0)
                    state = JoystickState.from_json(msg.data.decode("utf-8"))
                    await self.set_state(state)
                except nats.errors.TimeoutError:
                    continue
                except Exception as e:
                    self.get_logger().error(f'Error processing message: {str(e)}')
                    continue
        except Exception as e:
            self.get_logger().error(f'Error in NATS server: {str(e)}')
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
            node.get_logger().info('Shutting down...')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
