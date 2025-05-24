import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from manual.utils.joystick import JoystickState
import threading
import asyncio
import nats
from msgs.msg import Float32Stamped


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        self.wrench_publisher = self.create_publisher(WrenchStamped, 'wrench', 10)
        self.get_logger().info('Joystick node initialized')

        # Flag to control the async server
        self.running = True

        self.depth = 0.0
        self.last_depth = None
        self.depth_rate = 0.0
        self.last_depth_time = None
        self.desired_depth = None
        self.kp = 3  # Proportional gain
        self.kd = 1  # Derivative gain

        self.depth_subscription = self.create_subscription(
            Float32Stamped, "depth", self.depth_callback, 10
        )
        
        # Start the async server in a separate thread
        self.async_thread = threading.Thread(target=self._run_async_server, daemon=True)
        self.async_thread.start()

    def depth_callback(self, msg: Float32Stamped):
        current_depth = -msg.data
        now = self.get_clock().now()

        if self.last_depth is not None and self.last_depth_time is not None:
            delta_t = (now.nanoseconds - self.last_depth_time.nanoseconds) * 1e-9
            if delta_t > 0:
                self.depth_rate = (current_depth - self.last_depth) / delta_t

        self.last_depth_time = now
        self.last_depth = current_depth
        self.depth = current_depth

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
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = "base_link"

        if state.enabled:
            wrench_msg.wrench.force.x = state.ly * 0.5
            wrench_msg.wrench.force.y = -state.lx * 0.5

            # Handle depth control
            z_input = state.ry
            if abs(z_input) > 0.05:  # User wants to move up/down
                self.desired_depth = self.depth + z_input  # Move in increments
            elif self.desired_depth is None:
                self.desired_depth = self.depth

            # PD Controller
            error = self.desired_depth - self.depth
            d_error = -self.depth_rate
            force_z = self.kp * error + self.kd * d_error

            self.get_logger().info(f"Desired depth: {self.desired_depth}, Current depth: {self.depth}, {force_z}")

            wrench_msg.wrench.force.z = force_z

            wrench_msg.wrench.torque.x = 0.0
            wrench_msg.wrench.torque.y = 0.0
            wrench_msg.wrench.torque.z = -state.rx * 0.1
        else:
            wrench_msg.wrench.force.x = 0.0
            wrench_msg.wrench.force.y = 0.0
            wrench_msg.wrench.force.z = 0.0
            wrench_msg.wrench.torque.x = 0.0
            wrench_msg.wrench.torque.y = 0.0
            wrench_msg.wrench.torque.z = 0.0

        # self.get_logger().info(f"Publishing wrench: {wrench_msg.wrench}")

        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self.publish_wrench, wrench_msg)

    def publish_wrench(self, wrench_msg):
        self.wrench_publisher.publish(wrench_msg)

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