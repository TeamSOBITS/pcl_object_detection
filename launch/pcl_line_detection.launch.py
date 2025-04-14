import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # pcl_object_detection_src_dir = os.path.join(os.environ['HOME'], "colcon_ws", "src", "pcl_object_detection")
    pcl_object_detection_path = get_package_share_directory("pcl_object_detection")

    common_param_ = os.path.join(pcl_object_detection_src_dir,"param","common_param.yaml")
    line_param_ = os.path.join(pcl_object_detection_src_dir,"param","line_detection_param.yaml")
    floor_param_ = os.path.join(pcl_object_detection_src_dir,"param","object_detection_floor_param.yaml")
    shelf_param_ = os.path.join(pcl_object_detection_src_dir,"param","object_detection_shelf_param.yaml")
    table_param_ = os.path.join(pcl_object_detection_src_dir,"param","object_detection_table_param.yaml")
    placeable_param_ = os.path.join(pcl_object_detection_src_dir,"param","placeable_postion_detection_param.yaml")

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('manager_name', default_value='node_manager'),
        DeclareLaunchArgument('manager_threads', default_value='4'),
        DeclareLaunchArgument('node_start_delay', default_value='5.0'),
        DeclareLaunchArgument('rviz2', default_value='false'),
        DeclareLaunchArgument('rviz2_cfg', default_value=os.path.join(pcl_object_detection_src_dir, "config", "rviz", "pcl_object_detection.rviz")),
        DeclareLaunchArgument('rqt_reconfigure', default_value='false'),
     
        # Conditionally launch RViz
        Node(
            condition=IfCondition(LaunchConfiguration('rviz2')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', LaunchConfiguration('rviz2_cfg')],
            output='screen'
        ),

        # Group nodes under pcl_object_detection namespace
        Node(
            package='pcl_object_detection',
            executable='mode_manager_node',
            # name='mode_manager_node',
            parameters=[
                {
                    "common_param": common_param_,
                    "table_param": table_param_,
                    "shelf_param": shelf_param_,
                    "floor_param": floor_param_,
                    "placeable_param": placeable_param_,
                    "line_param": line_param_,
                }],
            output='screen'
        ),

    ])
