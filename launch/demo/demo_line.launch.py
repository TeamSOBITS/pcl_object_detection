from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the shared directories for the included launch files
    pcl_object_detection_dir = get_package_share_directory('pcl_object_detection')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('rviz', default_value='true', description='Enable RViz'),
        DeclareLaunchArgument('rqt_reconfigure', default_value='true', description='Enable rqt_reconfigure'),

        # Include scan_publisher.launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pcl_object_detection_dir, 'launch/demo/scan_publisher.launch.py')),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz')
            }.items()
        ),

        # Include line_detection.launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pcl_object_detection_dir, 'launch/line_detection.launch.py')),
            launch_arguments={
                'rviz': 'false',
                'rqt_reconfigure': LaunchConfiguration('rqt_reconfigure')
            }.items()
        )
    ])
