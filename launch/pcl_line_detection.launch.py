import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # pcl_object_detection_src_dir = os.path.join(os.environ['HOME'], "colcon_ws", "src", "pcl_object_detection")
    pcl_object_detection_pkg = get_package_share_directory("pcl_object_detection")

    line_param_ = os.path.join(pcl_object_detection_pkg,"param","line_param.yaml")
    common_param_ = os.path.join(pcl_object_detection_pkg,"param","object_common_param.yaml")

    return LaunchDescription([    
        # Group nodes under pcl_object_detection namespace
        Node(
            package='pcl_object_detection',
            executable='pcl_line_detection',
            name='pcl_line_detection',
            parameters=[common_param_,line_param_], # Corrected: parameters expects a list of paths or dictionaries
            output='screen'
        ),

    ])