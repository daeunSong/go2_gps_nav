from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('go2_gps_nav')
    
    # Path to the parameters file
    params_file = os.path.join(pkg_dir, 'config', 'navigation_params.yaml')
    
    # Create the node
    gps_navigation_node = Node(
        package='go2_gps_nav',
        executable='gps_navigation_node',
        name='gps_navigation_node',
        output='screen',
        parameters=[params_file]
    )
    
    return LaunchDescription([
        gps_navigation_node
    ])