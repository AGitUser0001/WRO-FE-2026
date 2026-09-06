from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hiway_odom',
            executable='odom_udp_node',
            name='odom_udp_node',
            output='screen',
            parameters=[{
                'udp_port': 9999,
                'wheelbase': 0.138,
                'max_steering_angle': 0.6066,
                'servo_center': 500,
                'mm_per_tick': 0.2418,  # pi * 65 / 844.8
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': True,
                'tf_publish_rate': 20.0,
            }],
        ),
    ])
