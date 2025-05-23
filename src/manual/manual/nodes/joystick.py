import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from pydantic import BaseModel
import threading
from typing import Optional
import uvicorn

class WrenchCommand(BaseModel):
    force_x: float = 0.0
    force_y: float = 0.0
    force_z: float = 0.0
    torque_x: float = 0.0
    torque_y: float = 0.0
    torque_z: float = 0.0

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        
        # Create publisher for wrench messages
        self.wrench_publisher = self.create_publisher(Wrench, 'wrench_command', 10)
        
        # Initialize FastAPI app
        self.app = FastAPI()
        
        # Add CORS middleware to allow requests from any origin
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        # Setup FastAPI routes
        self.setup_routes()
        
        # Start FastAPI server in a separate thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        self.get_logger().info('Joystick node initialized with FastAPI server')

    def setup_routes(self):
        @self.app.post("/wrench")
        async def set_wrench(command: WrenchCommand):
            try:
                wrench = Wrench()
                wrench.force.x = command.force_x
                wrench.force.y = command.force_y
                wrench.force.z = command.force_z
                wrench.torque.x = command.torque_x
                wrench.torque.y = command.torque_y
                wrench.torque.z = command.torque_z

                self.get_logger().info(f"Publishing wrench: {wrench}")
                
                self.wrench_publisher.publish(wrench)
                return {"status": "success"}
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))

    def run_server(self):
        """Run the FastAPI server"""
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
