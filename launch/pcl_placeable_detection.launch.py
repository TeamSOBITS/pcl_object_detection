import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('pcl_object_detection')

    preprocessor_default_config = os.path.join(pkg_share, 'config', 'preprocessor_component.yaml')
    placeable_default_config = os.path.join(pkg_share, 'config', 'placeable_detection_component.yaml')

    preprocessor_config = LaunchConfiguration('preprocessor_config')
    placeable_config = LaunchConfiguration('placeable_config')

    preprocessor_config_arg = DeclareLaunchArgument(
        'preprocessor_config',
        default_value=preprocessor_default_config,
        description='Full path to the preprocessor component parameters file to use'
    )
    placeable_config_arg = DeclareLaunchArgument(
        'placeable_config',
        default_value=placeable_default_config,
        description='Full path to the placeable detection component parameters file to use'
    )

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        description="Namespace for the nodes (String)",
        default_value="",
    )

    container = ComposableNodeContainer(
        name='pcl_detection_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # PreProcessor (Standard Component, always running)
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::PreProcessorComponent',
                name='preprocessor',
                namespace=namespace,
                parameters=[preprocessor_config],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Placeable Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::PlaceableDetectionComponent',
                name='placeable_detection',
                namespace=namespace,
                parameters=[placeable_config],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        preprocessor_config_arg,
        placeable_config_arg,
        namespace_cmd,
        container,
    ])
