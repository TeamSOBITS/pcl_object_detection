from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('output', default_value='screen', description='Output type for nodes'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/points2', description='PointCloud topic name'),
        DeclareLaunchArgument('target_frame', default_value='base_footprint', description='Target frame'),
        DeclareLaunchArgument('save_path', default_value='pcl_object_detection/pcd/', description='Path to save PCD files'),
        DeclareLaunchArgument('save_file', default_value='rack_obstacle_zoom', description='Save file name for PCD files'),
        DeclareLaunchArgument('rviz', default_value='true', description='Enable RViz'),
        DeclareLaunchArgument('rviz_cfg', default_value='pcl_object_detection/config/rviz/pcl_object_detection.rviz', description='Path to RViz configuration file'),
        DeclareLaunchArgument('rosbag', default_value='true', description='Enable rosbag play'),
        DeclareLaunchArgument('rosbag_cfg', default_value='/home/sobits/catkin_ws/src/rosbag/shelf_obstacle_zoom_2.bag', description='Path to rosbag file'),

        # Group for topic_template namespace
        GroupAction([
            Node(
                package='pcl_object_detection',
                executable='pcl_object_detection_save_pcd',
                name='save_pcd',
                output=LaunchConfiguration('output'),
                parameters=[
                    {'pointcloud_topic': LaunchConfiguration('pointcloud_topic')},
                    {'target_frame': LaunchConfiguration('target_frame')},
                    {'save_path': LaunchConfiguration('save_path')},
                    {'save_file': LaunchConfiguration('save_file')}
                ]
            )
        ]),

        # RViz node
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration('rviz_cfg')],
            output='screen'
        ),

        # Rosbag play node
        Node(
            condition=IfCondition(LaunchConfiguration('rosbag')),
            package='rosbag2',
            executable='ros2_bag_play',
            name='rosbag_play',
            arguments=[LaunchConfiguration('rosbag_cfg'), '--clock', '--loop'],
            output='screen'
        )
    ])
