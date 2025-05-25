from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths to package directories
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    
    # Create a container for the camera nodes
    container = ComposableNodeContainer(
        name='oak_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # First camera
            ComposableNode(
                package='depthai_ros_driver',
                plugin='depthai_ros_driver::Camera',
                name='oak1',
                parameters=[{
                    'name': 'oak1',
                    'camera_model': 'OAK-D',
                    'mxid': '19443010B17E1C1300',
                    'sync_nn': True,
                    'nn_type': 'mobilenet',
                    'enable_depth': True,
                    'lr_check': True,
                    'subpixel': True,
                    'extended_disparity': True,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Second camera
            ComposableNode(
                package='depthai_ros_driver',
                plugin='depthai_ros_driver::Camera',
                name='oak2',
                parameters=[{
                    'name': 'oak2',
                    'camera_model': 'OAK-D',
                    'mxid': '1944301021531E1300',
                    'sync_nn': True,
                    'nn_type': 'mobilenet',
                    'enable_depth': True,
                    'lr_check': True,
                    'subpixel': True,
                    'extended_disparity': True,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Rectify nodes
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color_node1',
                parameters=[{'camera_name': 'oak1'}],
                remappings=[
                    ('image', '/oak1/rgb/image_raw'),
                    ('camera_info', '/oak1/rgb/camera_info'),
                    ('image_rect', '/oak1/rgb/image_rect'),
                    ('image_rect/compressed', '/oak1/rgb/image_rect/compressed'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_color_node2',
                parameters=[{'camera_name': 'oak2'}],
                remappings=[
                    ('image', '/oak2/rgb/image_raw'),
                    ('camera_info', '/oak2/rgb/camera_info'),
                    ('image_rect', '/oak2/rgb/image_rect'),
                    ('image_rect/compressed', '/oak2/rgb/image_rect/compressed'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    
    # Camera Viewers

    camera_viewer = Node(
        package="perception",
        executable="camera_viewer",
        output="screen"
    )
    
    return LaunchDescription([
        container,
        camera_viewer
    ])