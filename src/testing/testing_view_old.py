import rclpy
from rclpy.node import Node
from typing import Dict, Any, List
import numpy as np
from collections import deque
import time
import plotly.graph_objs as go
from dash import Dash, dcc, html
from dash.dependencies import Input, Output, State
from threading import Thread, Lock
import json
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


class SubMonitorNode(Node):
    def __init__(self):
        super().__init__("sub_monitor")
        self.get_logger().info("Sub Monitor Dashboard has been created!")

        # Lock for thread-safe data access
        self.lock = Lock()
        
        # Store available topics and their types
        self.available_topics = {}
        self.active_publishers = set()
        
        # Time-series data storage (max 100 points)
        self.time_series_data = {
            'timestamps': deque(maxlen=100),
            'values': deque(maxlen=100)
        }
        
        # Current selected topic for graphing
        self.selected_topic = None
        self.selected_subscription = None
        
        # Discover topics
        self.discover_topics()
            
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
    app = Dash(__name__)
    
    # 4-panel grid layout
    app.layout = html.Div([
        html.H1("RoboSub Live Monitor Dashboard", style={'textAlign': 'center', 'marginBottom': '20px'}),
        
        # Grid container for 4 panels
        html.Div([
            # Top-left panel (placeholder for now)
            html.Div([
                html.H2("Panel 1 (Top-Left)"),
                html.P("Content coming soon...")
            ], style={
                'border': '2px solid #333',
                'padding': '20px',
                'height': '400px',
                'backgroundColor': '#f9f9f9'
            }),
            
            # Top-right panel (placeholder for now)
            html.Div([
                html.H2("Panel 2 (Top-Right)"),
                html.P("Content coming soon...")
            ], style={
                'border': '2px solid #333',
                'padding': '20px',
                'height': '400px',
                'backgroundColor': '#f9f9f9'
            }),
            
            # Bottom-left panel (placeholder for now)
            html.Div([
                html.H2("Panel 3 (Bottom-Left)"),
                html.P("Content coming soon...")
            ], style={
                'border': '2px solid #333',
                'padding': '20px',
                'height': '400px',
                'backgroundColor': '#f9f9f9'
            }),
            
            # Bottom-right panel - Time Series Graph
            html.Div([
                html.H2("Time-Series Topic Monitor"),
                html.Label("Select ROS2 Topic:"),
                dcc.Dropdown(
                    id='topic-dropdown',
                    options=[],
                    value=None,
                    placeholder="Select a topic..."
                ),
                dcc.Graph(
                    id='time-series-graph',
                    config={'displayModeBar': False},
                    style={'height': '300px'}
                )
            ], style={
                'border': '2px solid #333',
                'padding': '20px',
                'height': '400px',
                'backgroundColor': '#f9f9f9'
            }),
        ], style={
            'display': 'grid',
            'gridTemplateColumns': '1fr 1fr',
            'gridTemplateRows': '1fr 1fr',
            'gap': '20px',
            'padding': '20px'
        }),
        
        # Update intervals
        dcc.Interval(
            id='topic-list-interval',
            interval=5000,  # Update topic list every 5 seconds
            n_intervals=0
        ),
        dcc.Interval(
            id='graph-interval',
            interval=100,  # Update graph every 100ms
            n_intervals=0
        )
    ])

    # Callback to update topic dropdown
    @app.callback(
        Output('topic-dropdown', 'options'),
        [Input('topic-list-interval', 'n_intervals')]
    )
    def update_topic_list(n):
        # Refresh topic discovery
        node.discover_topics()
        
        options = []
        for topic_name in sorted(node.available_topics.keys()):
            # Check if topic has active publishers
            is_active = topic_name in node.active_publishers
            
            options.append({
                'label': f"{topic_name} {'✓' if is_active else '✗'}",
                'value': topic_name,
                'disabled': not is_active
            })
        
        return options
    
    # Callback when topic is selected
    @app.callback(
        Output('topic-dropdown', 'value'),
        [Input('topic-dropdown', 'value')],
        prevent_initial_call=True
    )
    def on_topic_selected(topic_name):
        if topic_name:
            node.subscribe_to_topic(topic_name)
        return topic_name
    
    # Callback to update time-series graph
    @app.callback(
        Output('time-series-graph', 'figure'),
        [Input('graph-interval', 'n_intervals')]
    )
    def update_graph(n):
        with node.lock:
            if len(node.time_series_data['timestamps']) == 0:
                # Empty graph
                return {
                    'data': [],
                    'layout': go.Layout(
                        title='No data yet - select a topic',
                        xaxis={'title': 'Time (s)'},
                        yaxis={'title': 'Value'},
                        margin={'l': 40, 'r': 20, 't': 40, 'b': 40}
                    )
                }
            
            # Convert timestamps to relative time
            timestamps = list(node.time_series_data['timestamps'])
            values = list(node.time_series_data['values'])
            
            if timestamps:
                start_time = timestamps[0]
                relative_times = [t - start_time for t in timestamps]
            else:
                relative_times = []
            
            return {
                'data': [
                    go.Scatter(
                        x=relative_times,
                        y=values,
                        mode='lines+markers',
                        name=node.selected_topic or 'Topic',
                        line={'color': '#1f77b4', 'width': 2},
                        marker={'size': 4}
                    )
                ],
                'layout': go.Layout(
                    title=f'Topic: {node.selected_topic or "None"}',
                    xaxis={'title': 'Time (s)'},
                    yaxis={'title': 'Value'},
                    margin={'l': 50, 'r': 20, 't': 40, 'b': 40},
                    hovermode='closest'
                )
            }

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
    
    app.run(debug=False, host="0.0.0.0", port=8052, use_reloader=False)
    

if __name__ == "__main__":
    main()
