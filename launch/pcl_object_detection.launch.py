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

    # ── One component_container_mt PER node (separate processes) ──────────────
    # Isolating each node in its own container + executor makes every lifecycle
    # subscription get serviced.
    def make_container(node_name, plugin, config):
        return ComposableNodeContainer(
            name=f'{node_name}_container',
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='pcl_object_detection',
                    plugin=plugin,
                    name=node_name,
                    namespace=namespace,
                    parameters=[config, {'use_sim_time': use_sim_time}],
                    extra_arguments=[{'use_intra_process_comms': False}],
                ),
            ],
            output='screen',
        )

    containers = [
        make_container('preprocessor',
                       'pcl_object_detection::PreProcessorComponent', preprocessor_config),
        make_container('table_detection',
                       'pcl_object_detection::TableDetectionComponent', table_config),
        make_container('floor_detection',
                       'pcl_object_detection::FloorDetectionComponent', floor_config),
        make_container('shelf_detection',
                       'pcl_object_detection::ShelfDetectionComponent', shelf_config),
        make_container('placeable_detection',
                       'pcl_object_detection::PlaceableDetectionComponent', placeable_config),
        make_container('line_detection',
                       'pcl_object_detection::LineDetectionComponent', line_config),
        make_container('basket_detection',
                       'pcl_object_detection::BasketDetectionComponent', basket_config),
        make_container('washing_machine_detection',
                       'pcl_object_detection::WashingMachineDetectionComponent', washing_machine_config),
        make_container('laundry_detection',
                       'pcl_object_detection::LaundryDetectionComponent', laundry_config),
    ]

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
        *containers,
    ])
