from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'octomap_path': '/home/ros2_ws/src/fr_079.ot',  # Leave empty for no pre-loading
                'resolution': 0.05,
                'max_range': 5.0,

                'publish_free_space': True,  # Важно для визуализации
                'latch': True,
                'pointcloud_min_z': 0.0,
                'pointcloud_max_z': 10.0,
            }],
            # remappings=[
            #     ('cloud_in', '/octomap/example'),
            # ]
        )
    ])