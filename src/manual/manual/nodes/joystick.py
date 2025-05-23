import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import threading
import asyncio
import uvicorn


class JoystickState(BaseModel):
    lx: float = 0.0  # Left joystick X
    ly: float = 0.0  # Left joystick Y
    rx: float = 0.0  # Right joystick X
    ry: float = 0.0  # Right joystick Y
    enabled: bool = False


class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')

        self.wrench_publisher = self.create_publisher(WrenchStamped, 'wrench', 10)

        self.app = FastAPI()
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        self.setup_routes()

        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.get_logger().info('Joystick node initialized with FastAPI server')

    def setup_routes(self):
        @self.app.post("/state")
        async def set_state(state: JoystickState):

            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = self.get_clock().now().to_msg()
            wrench_msg.header.frame_id = "base_link"

            if state.enabled:
                wrench_msg.wrench.force.x = state.ly * 0.05
                wrench_msg.wrench.force.y = state.lx * 0.05
                wrench_msg.wrench.force.z = state.ry * 0.05
                wrench_msg.wrench.torque.x = 0.0
                wrench_msg.wrench.torque.y = 0.0
                wrench_msg.wrench.torque.z = state.rx * 0.05
            else:
                wrench_msg.wrench.force.x = 0.0
                wrench_msg.wrench.force.y = 0.0
                wrench_msg.wrench.force.z = 0.0
                wrench_msg.wrench.torque.x = 0.0
                wrench_msg.wrench.torque.y = 0.0
                wrench_msg.wrench.torque.z = 0.0

            loop = asyncio.get_event_loop()

            try:
                await loop.run_in_executor(None, self.publish_wrench, wrench_msg)
                return {"status": "success"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

    def publish_wrench(self, wrench_msg):
        # self.get_logger().info(f"Publishing wrench: {wrench_msg}")
        self.wrench_publisher.publish(wrench_msg)

    def run_server(self):
        uvicorn.run(self.app, host="0.0.0.0", port=8000)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
