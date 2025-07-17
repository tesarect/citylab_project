from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_patrol')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'robot_patrol.rviz')
    
    return LaunchDescription([
        # patrol node
        Node(
            package='robot_patrol',
            executable='patrol_bot_node',
            output='screen',
            emulate_tty=True
        ),
        
        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])

    # rviz_args = ['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    
    # return LaunchDescription([
    #     Node(package='robot_patrol', executable='patrol_bot_node', output='screen'),
    #     Node(package='rviz2', executable='rviz2', arguments=rviz_args, output='screen')
    # ])