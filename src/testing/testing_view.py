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


class SubMonitorNode(Node):
    def __init__(self):
        super().__init__("sub_monitor")
        self.get_logger().info("Sub Monitor Dashboard has been created!")

        # Lock for thread-safe data access
        self.lock = Lock()
        
        # Store available topics and their types
        self.available_topics = {}
        self.active_publishers = set()
        
        # Time-series data storage (10 seconds of data at ~100Hz = 1000 points)
        self.time_series_data = {
            'timestamps': deque(maxlen=1000),
            'values': deque(maxlen=1000)
        }
        
        # Track start time for relative timestamps
        self.graph_start_time = None
        
        # Current selected topic for graphing
        self.selected_topic = None
        self.selected_subscription = None
        
        # Discover topics
        self.discover_topics()
    
    def discover_topics(self):
        """Discover all available topics and their types"""
        topic_list = self.get_topic_names_and_types()
        
        for topic_name, topic_types in topic_list:
            if topic_types:
                self.available_topics[topic_name] = topic_types[0]
                
                # Check if this topic has active publishers
                try:
                    publishers = self.get_publishers_info_by_topic(topic_name)
                    if len(publishers) > 0:
                        self.active_publishers.add(topic_name)
                except:
                    pass
        
        self.get_logger().info(f"Discovered {len(self.available_topics)} topics")
        self.get_logger().info(f"Active publishers on {len(self.active_publishers)} topics")
    
    def subscribe_to_topic(self, topic_name: str):
        """Subscribe to a topic for time-series graphing"""
        if topic_name not in self.available_topics:
            self.get_logger().warn(f"Topic {topic_name} not found")
            return
        
        # Unsubscribe from previous topic if exists
        if self.selected_subscription:
            self.destroy_subscription(self.selected_subscription)
        
        topic_type = self.available_topics[topic_name]
        
        # Clear previous data
        with self.lock:
            self.time_series_data['timestamps'].clear()
            self.time_series_data['values'].clear()
            self.selected_topic = topic_name
        
        # Create generic callback
        def generic_callback(msg):
            with self.lock:
                current_time = time.time()
                if self.graph_start_time is None:
                    self.graph_start_time = current_time
                self.time_series_data['timestamps'].append(current_time)
                # Extract first numeric field from message
                value = self.extract_numeric_value(msg)
                self.time_series_data['values'].append(value)
        
        # Create subscription
        try:
            msg_class = get_message(topic_type)
            
            self.selected_subscription = self.create_subscription(
                msg_class,
                topic_name,
                generic_callback,
                10
            )
            self.get_logger().info(f"Subscribed to {topic_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to subscribe to {topic_name}: {e}")
    
    def extract_numeric_value(self, msg) -> float:
        """Extract a numeric value from a ROS message"""
        # Try to find first numeric field
        for field in msg.get_fields_and_field_types():
            try:
                value = getattr(msg, field)
                if isinstance(value, (int, float)):
                    return float(value)
                # Handle nested messages (e.g., pose.position.x)
                if hasattr(value, 'x'):
                    return float(value.x)
                if hasattr(value, 'data'):
                    return float(value.data)
            except:
                continue
        return 0.0
    
    def reset_graph_data(self):
        """Reset the graph data"""
        with self.lock:
            self.time_series_data['timestamps'].clear()
            self.time_series_data['values'].clear()
            self.graph_start_time = None


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
                html.Div([
                    html.H2("Time-Series Topic Monitor", style={'display': 'inline-block', 'marginRight': '20px'}),
                    html.Button('Reset Graph', id='reset-button', n_clicks=0, style={
                        'backgroundColor': '#ff4444',
                        'color': 'white',
                        'border': 'none',
                        'padding': '8px 16px',
                        'borderRadius': '4px',
                        'cursor': 'pointer',
                        'fontSize': '14px'
                    })
                ], style={'marginBottom': '10px'}),
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
                    style={'height': '280px'}
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
    
    # Callback for reset button
    @app.callback(
        Output('reset-button', 'n_clicks'),
        [Input('reset-button', 'n_clicks')],
        prevent_initial_call=True
    )
    def reset_graph(n_clicks):
        if n_clicks > 0:
            node.reset_graph_data()
        return 0
    
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
                        xaxis={'title': 'Time (s)', 'range': [-10, 0]},
                        yaxis={'title': 'Value'},
                        margin={'l': 50, 'r': 20, 't': 40, 'b': 40}
                    )
                }
            
            # Get current time and filter to last 10 seconds
            current_time = time.time()
            timestamps = list(node.time_series_data['timestamps'])
            values = list(node.time_series_data['values'])
            
            # Filter to last 10 seconds
            cutoff_time = current_time - 10.0
            filtered_data = [(t, v) for t, v in zip(timestamps, values) if t >= cutoff_time]
            
            if not filtered_data:
                return {
                    'data': [],
                    'layout': go.Layout(
                        title=f'Topic: {node.selected_topic or "None"} (waiting for data...)',
                        xaxis={'title': 'Time (s)', 'range': [-10, 0]},
                        yaxis={'title': 'Value'},
                        margin={'l': 50, 'r': 20, 't': 40, 'b': 40}
                    )
                }
            
            filtered_timestamps, filtered_values = zip(*filtered_data)
            
            # Convert to relative time (negative values, 0 = now)
            relative_times = [t - current_time for t in filtered_timestamps]
            
            # Auto-scale Y-axis with some padding
            y_min = min(filtered_values)
            y_max = max(filtered_values)
            y_range = y_max - y_min
            y_padding = y_range * 0.1 if y_range > 0 else 1.0
            
            return {
                'data': [
                    go.Scatter(
                        x=relative_times,
                        y=filtered_values,
                        mode='lines',
                        name=node.selected_topic or 'Topic',
                        line={'color': '#1f77b4', 'width': 2}
                    )
                ],
                'layout': go.Layout(
                    title=f'Topic: {node.selected_topic or "None"}',
                    xaxis={
                        'title': 'Time (s)',
                        'range': [-10, 0],
                        'fixedrange': True
                    },
                    yaxis={
                        'title': 'Value',
                        'range': [y_min - y_padding, y_max + y_padding],
                        'autorange': False
                    },
                    margin={'l': 50, 'r': 20, 't': 40, 'b': 40},
                    hovermode='closest',
                    shapes=[
                        # Red vertical line at present moment (x=0)
                        {
                            'type': 'line',
                            'x0': 0,
                            'x1': 0,
                            'y0': y_min - y_padding,
                            'y1': y_max + y_padding,
                            'line': {
                                'color': 'red',
                                'width': 2,
                                'dash': 'dash'
                            }
                        }
                    ]
                )
            }

    return app


def ros2_thread(node):
    """Run ROS2 in a separate thread"""
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    rclpy.init()
    
    # Create ROS2 node
    node = SubMonitorNode()
    
    # Create Dash app
    app = create_dash_app(node)
    
    # Run ROS2 spin in a separate thread
    ros_thread = Thread(target=ros2_thread, args=(node,))
    ros_thread.daemon = True
    ros_thread.start()

    # Run Dash app on port 8052
    print("\n" + "="*60)
    print("RoboSub Monitor Dashboard Starting...")
    print("Open your browser to: http://localhost:8052")
    print("="*60 + "\n")
    
    app.run(debug=False, host="0.0.0.0", port=8052, use_reloader=False)


if __name__ == "__main__":
    main()
