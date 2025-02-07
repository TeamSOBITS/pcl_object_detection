from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('detection_mode', default_value='1', description='Detection mode: OFF=0, TABLE_MODE=1, FLOOR_MODE=2, SHELF_MODE=3, PLACEABLE_POSITION=4'),
        DeclareLaunchArgument('rqt_reconfigure', default_value='true', description='Enable rqt_reconfigure'),
        DeclareLaunchArgument('rviz', default_value='true', description='Enable RViz'),
        DeclareLaunchArgument('rviz_cfg', default_value='pcl_object_detection/config/rviz/demo.rviz', description='Path to RViz configuration file'),

        # RViz node
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration('rviz_cfg')],
            output='screen'
        ),

        # Pointcloud publisher node
        Node(
            package='pcl_object_detection',
            executable='pcl_object_detection_pointcloud_publisher',
            name='pointcloud_publisher',
            output='screen',
            parameters=[
                {'detection_mode': LaunchConfiguration('detection_mode')},
                {'table_pcd_path': 'pcl_object_detection/pcd/table_object_binary.pcd'},
                {'floor_pcd_path': 'pcl_object_detection/pcd/floor_object_binary.pcd'},
                {'shelf_pcd_path': 'pcl_object_detection/pcd/shelf_object_binary.pcd'},
                {'placeable_pcd_path': 'pcl_object_detection/pcd/table_object_binary.pcd'}
            ]
        ),

        # Static transform publisher node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot2senser',
            arguments=['0', '0', '1.35', '-1.54', '0.0', '-2.0', 'base_footprint', 'camera_link'],
            output='screen'
        ),

        # rqt_reconfigure node
        Node(
            condition=IfCondition(LaunchConfiguration('rqt_reconfigure')),
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            name='rqt_reconfigure',
            output='screen'
        )
    ])
