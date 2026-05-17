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
    basket_default_config = os.path.join(pkg_share, 'config', 'basket_detection_component.yaml')

    preprocessor_config = LaunchConfiguration('preprocessor_config')
    basket_config = LaunchConfiguration('basket_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    preprocessor_config_arg = DeclareLaunchArgument(
        'preprocessor_config',
        default_value=preprocessor_default_config,
        description='Full path to the preprocessor component parameters file to use'
    )
    basket_config_arg = DeclareLaunchArgument(
        'basket_config',
        default_value=basket_default_config,
        description='Full path to the basket detection component parameters file to use'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        description="Namespace for the nodes (String)",
        default_value="",
    )

    container = ComposableNodeContainer(
        name='pcl_basket_detection_container',
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
                parameters=[preprocessor_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': False}]
            ),
            # Basket Detection Worker (Lifecycle Node)
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::BasketDetectionComponent',
                name='basket_detection',
                namespace=namespace,
                parameters=[basket_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': False}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        preprocessor_config_arg,
        basket_config_arg,
        use_sim_time_arg,
        namespace_cmd,
        container,
    ])
