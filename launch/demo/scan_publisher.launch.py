from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('rviz', default_value='true', description='Enable RViz'),
        DeclareLaunchArgument('rviz_cfg', default_value='pcl_object_detection/config/rviz/line_detection.rviz', description='Path to RViz configuration file'),

        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot2senser',
            arguments=['0.05', '-0.1', '0.3', '0', '0', '0', 'base_footprint', 'base_laser_link', '100'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map2robot',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint', '100'],
            output='screen'
        ),

        # RViz node
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration('rviz_cfg')],
            output='screen'
        ),

        # Scan publisher group
        GroupAction([
            Node(
                package='pcl_object_detection',
                executable='pcl_object_detection_scan_publisher',
                name='scan_publisher',
                output='screen'
            )
        ]),

        # Pointcloud to laserscan node
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/cloud_laserscan')
            ],
            parameters=[{
                'target_frame': 'base_laser_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': 0.0087,
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'concurrency_level': 1
            }],
            output='screen'
        )
    ])
