import rclpy
from rclpy.node import Node
from typing import Dict, Any
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32
from dash import Dash, dcc, html
from dash.dependencies import Input, Output
from threading import Thread
from transforms3d.euler import quat2euler
import json


class SubMonitorNode(Node):
    def __init__(self):
        super().__init__("sub_monitor")
        self.get_logger().info("Sub Monitor Dashboard has been created!")

        # Shared data dictionary for all sensor/node values
        self.data = {
            # Odometry data
            "position_x": 0.0,
            "position_y": 0.0,
            "position_z": 0.0,
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "angular_vel_x": 0.0,
            "angular_vel_y": 0.0,
            "angular_vel_z": 0.0,
            
            # IMU data
            "imu_accel_x": 0.0,
            "imu_accel_y": 0.0,
            "imu_accel_z": 0.0,
            "imu_gyro_x": 0.0,
            "imu_gyro_y": 0.0,
            "imu_gyro_z": 0.0,
            
            # Detection data
            "num_detections": 0,
            "detection_labels": [],
            "detection_distances": [],
            
            # Control data
            "wrench_force_x": 0.0,
            "wrench_force_y": 0.0,
            "wrench_force_z": 0.0,
            "wrench_torque_x": 0.0,
            "wrench_torque_y": 0.0,
            "wrench_torque_z": 0.0,
            
            # Depth
            "depth": 0.0,
            
            # Status
            "last_update": "Never",
        }

        # Create subscriptions to all critical topics
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10
        )
        
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, 10
        )
        
        self.detections_sub = self.create_subscription(
            Detection3DArray, "/detections3d", self.detections_callback, 10
        )
        
        self.wrench_sub = self.create_subscription(
            WrenchStamped, "/wrench", self.wrench_callback, 10
        )
        
        self.depth_sub = self.create_subscription(
            Float32, "/depth", self.depth_callback, 10
        )

    def odom_callback(self, msg: Odometry):
        """Process odometry data"""
        # Position
        self.data["position_x"] = msg.pose.pose.position.x
        self.data["position_y"] = msg.pose.pose.position.y
        self.data["position_z"] = msg.pose.pose.position.z
        
        # Velocity
        self.data["velocity_x"] = msg.twist.twist.linear.x
        self.data["velocity_y"] = msg.twist.twist.linear.y
        self.data["velocity_z"] = msg.twist.twist.linear.z
        
        # Angular velocity
        self.data["angular_vel_x"] = msg.twist.twist.angular.x
        self.data["angular_vel_y"] = msg.twist.twist.angular.y
        self.data["angular_vel_z"] = msg.twist.twist.angular.z
        
        # Orientation (convert quaternion to euler)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        roll, pitch, yaw = quat2euler([qw, qx, qy, qz])
        
        self.data["roll"] = np.degrees(roll)
        self.data["pitch"] = np.degrees(pitch)
        self.data["yaw"] = np.degrees(yaw)
        
        self.data["last_update"] = "Odometry"

    def imu_callback(self, msg: Imu):
        """Process IMU data"""
        self.data["imu_accel_x"] = msg.linear_acceleration.x
        self.data["imu_accel_y"] = msg.linear_acceleration.y
        self.data["imu_accel_z"] = msg.linear_acceleration.z
        
        self.data["imu_gyro_x"] = msg.angular_velocity.x
        self.data["imu_gyro_y"] = msg.angular_velocity.y
        self.data["imu_gyro_z"] = msg.angular_velocity.z

    def detections_callback(self, msg: Detection3DArray):
        """Process detection data"""
        self.data["num_detections"] = len(msg.detections)
        
        labels = []
        distances = []
        for detection in msg.detections:
            if detection.results:
                label = detection.results[0].hypothesis.class_id
                labels.append(label)
            
            # Calculate distance from center
            center = detection.bbox.center
            distance = np.sqrt(
                center.position.x**2 + 
                center.position.y**2 + 
                center.position.z**2
            )
            distances.append(round(distance, 2))
        
        self.data["detection_labels"] = labels
        self.data["detection_distances"] = distances

    def wrench_callback(self, msg: WrenchStamped):
        """Process control wrench data"""
        self.data["wrench_force_x"] = msg.wrench.force.x
        self.data["wrench_force_y"] = msg.wrench.force.y
        self.data["wrench_force_z"] = msg.wrench.force.z
        
        self.data["wrench_torque_x"] = msg.wrench.torque.x
        self.data["wrench_torque_y"] = msg.wrench.torque.y
        self.data["wrench_torque_z"] = msg.wrench.torque.z

    def depth_callback(self, msg: Float32):
        """Process depth sensor data"""
        self.data["depth"] = msg.data


def create_dash_app(node: SubMonitorNode):
    """Create the Dash web application"""
    app = Dash(__name__)

    app.layout = html.Div([
        html.H1("RoboSub Live Monitor Dashboard", 
                style={"textAlign": "center", "color": "#2c3e50"}),
        
        html.Div([
            html.H3("Last Update: ", style={"display": "inline"}),
            html.Span(id="last-update", style={"color": "#27ae60"})
        ], style={"textAlign": "center", "marginBottom": "20px"}),
        
        # Position & Orientation Section
        html.Div([
            html.H2("Position & Orientation", style={"color": "#34495e"}),
            html.Div([
                html.Div([
                    html.H4("Position (m)"),
                    html.Table([
                        html.Tr([html.Td("X:"), html.Td(id="pos-x", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Y:"), html.Td(id="pos-y", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Z:"), html.Td(id="pos-z", style={"fontWeight": "bold"})]),
                    ])
                ], style={"display": "inline-block", "margin": "10px", "padding": "15px", "border": "1px solid #bdc3c7"}),
                
                html.Div([
                    html.H4("Orientation (deg)"),
                    html.Table([
                        html.Tr([html.Td("Roll:"), html.Td(id="roll", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Pitch:"), html.Td(id="pitch", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Yaw:"), html.Td(id="yaw", style={"fontWeight": "bold"})]),
                    ])
                ], style={"display": "inline-block", "margin": "10px", "padding": "15px", "border": "1px solid #bdc3c7"}),
                
                html.Div([
                    html.H4("Velocity (m/s)"),
                    html.Table([
                        html.Tr([html.Td("Vx:"), html.Td(id="vel-x", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Vy:"), html.Td(id="vel-y", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Vz:"), html.Td(id="vel-z", style={"fontWeight": "bold"})]),
                    ])
                ], style={"display": "inline-block", "margin": "10px", "padding": "15px", "border": "1px solid #bdc3c7"}),
            ], style={"textAlign": "center"})
        ], style={"marginBottom": "30px"}),
        
        # IMU Section
        html.Div([
            html.H2("IMU Sensors", style={"color": "#34495e"}),
            html.Div([
                html.Div([
                    html.H4("Acceleration (m/s²)"),
                    html.Table([
                        html.Tr([html.Td("Ax:"), html.Td(id="accel-x", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Ay:"), html.Td(id="accel-y", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Az:"), html.Td(id="accel-z", style={"fontWeight": "bold"})]),
                    ])
                ], style={"display": "inline-block", "margin": "10px", "padding": "15px", "border": "1px solid #bdc3c7"}),
                
                html.Div([
                    html.H4("Angular Velocity (rad/s)"),
                    html.Table([
                        html.Tr([html.Td("ωx:"), html.Td(id="gyro-x", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("ωy:"), html.Td(id="gyro-y", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("ωz:"), html.Td(id="gyro-z", style={"fontWeight": "bold"})]),
                    ])
                ], style={"display": "inline-block", "margin": "10px", "padding": "15px", "border": "1px solid #bdc3c7"}),
            ], style={"textAlign": "center"})
        ], style={"marginBottom": "30px"}),
        
        # Detections Section
        html.Div([
            html.H2("Object Detections", style={"color": "#34495e"}),
            html.Div([
                html.H4("Number of Detections: ", style={"display": "inline"}),
                html.Span(id="num-detections", style={"fontSize": "24px", "fontWeight": "bold", "color": "#e74c3c"}),
            ], style={"textAlign": "center"}),
            html.Div(id="detection-list", style={"textAlign": "center", "marginTop": "10px"})
        ], style={"marginBottom": "30px"}),
        
        # Control Section
        html.Div([
            html.H2("Control Wrench", style={"color": "#34495e"}),
            html.Div([
                html.Div([
                    html.H4("Force (N)"),
                    html.Table([
                        html.Tr([html.Td("Fx:"), html.Td(id="force-x", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Fy:"), html.Td(id="force-y", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Fz:"), html.Td(id="force-z", style={"fontWeight": "bold"})]),
                    ])
                ], style={"display": "inline-block", "margin": "10px", "padding": "15px", "border": "1px solid #bdc3c7"}),
                
                html.Div([
                    html.H4("Torque (Nm)"),
                    html.Table([
                        html.Tr([html.Td("Tx:"), html.Td(id="torque-x", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Ty:"), html.Td(id="torque-y", style={"fontWeight": "bold"})]),
                        html.Tr([html.Td("Tz:"), html.Td(id="torque-z", style={"fontWeight": "bold"})]),
                    ])
                ], style={"display": "inline-block", "margin": "10px", "padding": "15px", "border": "1px solid #bdc3c7"}),
            ], style={"textAlign": "center"})
        ], style={"marginBottom": "30px"}),
        
        # Depth Section
        html.Div([
            html.H2("Depth Sensor", style={"color": "#34495e"}),
            html.Div([
                html.H4("Depth: ", style={"display": "inline"}),
                html.Span(id="depth", style={"fontSize": "24px", "fontWeight": "bold", "color": "#3498db"}),
                html.Span(" m", style={"fontSize": "18px"}),
            ], style={"textAlign": "center"})
        ], style={"marginBottom": "30px"}),
        
        # Update interval
        dcc.Interval(
            id="update-interval",
            interval=500,  # Update every 500ms
            n_intervals=0
        ),
    ], style={"padding": "20px", "fontFamily": "Arial, sans-serif"})

    @app.callback(
        [
            Output("last-update", "children"),
            Output("pos-x", "children"),
            Output("pos-y", "children"),
            Output("pos-z", "children"),
            Output("roll", "children"),
            Output("pitch", "children"),
            Output("yaw", "children"),
            Output("vel-x", "children"),
            Output("vel-y", "children"),
            Output("vel-z", "children"),
            Output("accel-x", "children"),
            Output("accel-y", "children"),
            Output("accel-z", "children"),
            Output("gyro-x", "children"),
            Output("gyro-y", "children"),
            Output("gyro-z", "children"),
            Output("num-detections", "children"),
            Output("detection-list", "children"),
            Output("force-x", "children"),
            Output("force-y", "children"),
            Output("force-z", "children"),
            Output("torque-x", "children"),
            Output("torque-y", "children"),
            Output("torque-z", "children"),
            Output("depth", "children"),
        ],
        [Input("update-interval", "n_intervals")],
    )
    def update_dashboard(n):
        """Update all dashboard values"""
        data = node.data
        
        # Format detection list
        detection_items = []
        if data["num_detections"] > 0:
            for i, (label, dist) in enumerate(zip(data["detection_labels"], data["detection_distances"])):
                detection_items.append(
                    html.Div(f"Detection {i+1}: {label} at {dist}m", 
                            style={"margin": "5px", "padding": "5px", "backgroundColor": "#ecf0f1"})
                )
        else:
            detection_items = [html.Div("No detections", style={"color": "#95a5a6"})]
        
        return [
            data["last_update"],
            f"{data['position_x']:.3f}",
            f"{data['position_y']:.3f}",
            f"{data['position_z']:.3f}",
            f"{data['roll']:.2f}",
            f"{data['pitch']:.2f}",
            f"{data['yaw']:.2f}",
            f"{data['velocity_x']:.3f}",
            f"{data['velocity_y']:.3f}",
            f"{data['velocity_z']:.3f}",
            f"{data['imu_accel_x']:.3f}",
            f"{data['imu_accel_y']:.3f}",
            f"{data['imu_accel_z']:.3f}",
            f"{data['imu_gyro_x']:.3f}",
            f"{data['imu_gyro_y']:.3f}",
            f"{data['imu_gyro_z']:.3f}",
            str(data["num_detections"]),
            detection_items,
            f"{data['wrench_force_x']:.3f}",
            f"{data['wrench_force_y']:.3f}",
            f"{data['wrench_force_z']:.3f}",
            f"{data['wrench_torque_x']:.3f}",
            f"{data['wrench_torque_y']:.3f}",
            f"{data['wrench_torque_z']:.3f}",
            f"{data['depth']:.3f}",
        ]

    return app


def ros2_thread(node):
    """Run ROS2 in a separate thread"""
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Create ROS2 node
    node = SubMonitorNode()

    # Create Dash app
    app = create_dash_app(node)
    app.config["suppress_callback_exceptions"] = True

    # Run ROS2 spin in a separate thread
    ros_thread = Thread(target=ros2_thread, args=(node,))
    ros_thread.daemon = True
    ros_thread.start()

    # Run Dash app on port 8052 (different from view_detections_3d)
    print("\n" + "="*60)
    print("RoboSub Monitor Dashboard Starting...")
    print("Open your browser to: http://localhost:8052")
    print("="*60 + "\n")
    
    app.run_server(debug=False, host="0.0.0.0", port=8052, use_reloader=False)


if __name__ == "__main__":
    main()
