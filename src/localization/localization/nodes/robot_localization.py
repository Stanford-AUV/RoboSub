from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


#launches EKF node in robot_localization with parameters from sensors.yaml
def generate_launch_description():
    pkg_share = get_package_share_directory('localization')
        
    # Standard: Join paths to ensure absolute path resolution
    ekf_config = os.path.join(pkg_share, 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        )
    ])