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
    shelf_default_config = os.path.join(pkg_share, 'config', 'shelf_detection_component.yaml')

    preprocessor_config = LaunchConfiguration('preprocessor_config')
    shelf_config = LaunchConfiguration('shelf_config')

    preprocessor_config_arg = DeclareLaunchArgument(
        'preprocessor_config',
        default_value=preprocessor_default_config,
        description='Full path to the preprocessor component parameters file to use'
    )
    shelf_config_arg = DeclareLaunchArgument(
        'shelf_config',
        default_value=shelf_default_config,
        description='Full path to the shelf detection component parameters file to use'
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
            # Shelf Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::ShelfDetectionComponent',
                name='shelf_detection',
                namespace=namespace,
                parameters=[shelf_config],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        preprocessor_config_arg,
        shelf_config_arg,
        namespace_cmd,
        container,
    ])
