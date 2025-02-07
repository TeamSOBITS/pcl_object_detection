from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the shared directory for pcl_object_detection
    pcl_object_detection_dir = get_package_share_directory('pcl_object_detection')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('detection_mode', default_value='1', description='Detection mode: 1=TABLE_MODE, 2=FLOOR_MODE, 3=SHELF_MODE, 4=PLACEABLE_POSITION'),
        DeclareLaunchArgument('table_param', default_value=os.path.join(pcl_object_detection_dir, 'param/demo/table_demo_param.yaml')),
        DeclareLaunchArgument('floor_param', default_value=os.path.join(pcl_object_detection_dir, 'param/demo/floor_demo_param.yaml')),
        DeclareLaunchArgument('shelf_param', default_value=os.path.join(pcl_object_detection_dir, 'param/demo/shelf_demo_param.yaml')),
        DeclareLaunchArgument('placeable_param', default_value=os.path.join(pcl_object_detection_dir, 'param/demo/placeable_demo_param.yaml')),
        DeclareLaunchArgument('rviz', default_value='true', description='Enable RViz'),

        # Include pointcloud_publisher.launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pcl_object_detection_dir, 'launch/demo/pointcloud_publisher.launch.py')),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'),
                'rqt_reconfigure': 'false',
                'detection_mode': LaunchConfiguration('detection_mode')
            }.items()
        ),

        # Include pcl_object_detection.launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pcl_object_detection_dir, 'launch/pcl_object_detection.launch.py')),
            launch_arguments={
                'node_start_delay': '5.0',
                'rviz': 'false',
                'rqt_reconfigure': 'true',
                'detection_mode': LaunchConfiguration('detection_mode'),
                'table_param': LaunchConfiguration('table_param'),
                'floor_param': LaunchConfiguration('floor_param'),
                'shelf_param': LaunchConfiguration('shelf_param'),
                'placeable_param': LaunchConfiguration('placeable_param')
            }.items()
        )
    ])
