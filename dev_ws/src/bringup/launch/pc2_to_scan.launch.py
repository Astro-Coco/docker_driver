from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_scan = LaunchConfiguration('use_scan', default='true')

    return LaunchDescription([

        # allow turning scan on/off
        DeclareLaunchArgument(
            'use_scan',
            default_value='true',
            description='Whether to launch the pointcloud_to_laserscan node'
        ),

        # static transform from map → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_base_link',
            arguments=[
                '0', '0', '0',  '0', '0', '0', '1',
                'map', 'base_link'
            ],
        ),

        # pointcloud_to_laserscan, only if use_scan == true
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            condition=IfCondition(use_scan),
            remappings=[
                ('cloud_in', '/registered_scan'),
                ('scan',    '/scan'),
            ],
            parameters=[{
                'target_frame':       'base_link',
                'transform_tolerance': 0.1,
                'min_height':         0.0,
                'max_height':         1.0,
                'angle_min':         -3.141592,
                'angle_max':          3.141592,
                'angle_increment':    0.0087,
                'scan_time':          0.3333,
                'range_min':          0.20,
                'range_max':          4.0,
                'use_inf':            True,
                'inf_epsilon':        1.0,
            }],
        ),

    ])
