import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('pcl_object_detection')

    default_config = os.path.join(
        pkg_share_dir, 'config', 'cloth_keypoint_detector_component.yaml'
    )

    config_file  = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace    = LaunchConfiguration('namespace')

    container = ComposableNodeContainer(
        name='cloth_keypoint_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::ClothKeypointDetectorComponent',
                name='cloth_keypoint_detector',
                namespace=namespace,
                parameters=[config_file, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to cloth_keypoint_detector_component.yaml',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the node',
        ),
        container,
    ])
