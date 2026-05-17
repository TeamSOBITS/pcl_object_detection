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
    table_default_config = os.path.join(pkg_share, 'config', 'table_detection_component.yaml')
    floor_default_config = os.path.join(pkg_share, 'config', 'floor_detection_component.yaml')
    shelf_default_config = os.path.join(pkg_share, 'config', 'shelf_detection_component.yaml')
    placeable_default_config = os.path.join(pkg_share, 'config', 'placeable_detection_component.yaml')
    line_default_config = os.path.join(pkg_share, 'config', 'line_detection_component.yaml')
    basket_default_config = os.path.join(pkg_share, 'config', 'basket_detection_component.yaml')
    washing_machine_default_config = os.path.join(pkg_share, 'config', 'washing_machine_detection_component.yaml')
    laundry_default_config = os.path.join(pkg_share, 'config', 'laundry_detection_component.yaml')

    preprocessor_config = LaunchConfiguration('preprocessor_config')
    table_config = LaunchConfiguration('table_config')
    floor_config = LaunchConfiguration('floor_config')
    shelf_config = LaunchConfiguration('shelf_config')
    placeable_config = LaunchConfiguration('placeable_config')
    line_config = LaunchConfiguration('line_config')
    basket_config = LaunchConfiguration('basket_config')
    washing_machine_config = LaunchConfiguration('washing_machine_config')
    laundry_config = LaunchConfiguration('laundry_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    preprocessor_config_arg = DeclareLaunchArgument(
        'preprocessor_config',
        default_value=preprocessor_default_config,
        description='Full path to the preprocessor component parameters file to use'
    )
    table_config_arg = DeclareLaunchArgument(
        'table_config',
        default_value=table_default_config,
        description='Full path to the table detection component parameters file to use'
    )
    floor_config_arg = DeclareLaunchArgument(
        'floor_config',
        default_value=floor_default_config,
        description='Full path to the floor detection component parameters file to use'
    )
    shelf_config_arg = DeclareLaunchArgument(
        'shelf_config',
        default_value=shelf_default_config,
        description='Full path to the shelf detection component parameters file to use'
    )
    placeable_config_arg = DeclareLaunchArgument(
        'placeable_config',
        default_value=placeable_default_config,
        description='Full path to the placeable detection component parameters file to use'
    )
    line_config_arg = DeclareLaunchArgument(
        'line_config',
        default_value=line_default_config,
        description='Full path to the line detection component parameters file to use'
    )
    basket_config_arg = DeclareLaunchArgument(
        'basket_config',
        default_value=basket_default_config,
        description='Full path to the basket detection component parameters file to use'
    )
    washing_machine_config_arg = DeclareLaunchArgument(
        'washing_machine_config',
        default_value=washing_machine_default_config,
        description='Full path to the washing machine detection component parameters file to use'
    )
    laundry_config_arg = DeclareLaunchArgument(
        'laundry_config',
        default_value=laundry_default_config,
        description='Full path to the laundry detection component parameters file to use'
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
                parameters=[preprocessor_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Table Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::TableDetectionComponent',
                name='table_detection',
                namespace=namespace,
                parameters=[table_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Floor Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::FloorDetectionComponent',
                name='floor_detection',
                namespace=namespace,
                parameters=[floor_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Shelf Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::ShelfDetectionComponent',
                name='shelf_detection',
                namespace=namespace,
                parameters=[shelf_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Placeable Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::PlaceableDetectionComponent',
                name='placeable_detection',
                namespace=namespace,
                parameters=[placeable_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Line Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::LineDetectionComponent',
                name='line_detection',
                namespace=namespace,
                parameters=[line_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Basket Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::BasketDetectionComponent',
                name='basket_detection',
                namespace=namespace,
                parameters=[basket_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Washing Machine Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::WashingMachineDetectionComponent',
                name='washing_machine_detection',
                namespace=namespace,
                parameters=[washing_machine_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Laundry Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::LaundryDetectionComponent',
                name='laundry_detection',
                namespace=namespace,
                parameters=[laundry_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        preprocessor_config_arg,
        table_config_arg,
        floor_config_arg,
        shelf_config_arg,
        placeable_config_arg,
        line_config_arg,
        basket_config_arg,
        washing_machine_config_arg,
        laundry_config_arg,
        use_sim_time_arg,
        namespace_cmd,
        container,
    ])
