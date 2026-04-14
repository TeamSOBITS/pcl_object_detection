import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('pcl_object_detection')

    # Default paths
    default_config_file = os.path.join(pkg_share_dir, 'config', 'laundry_detection_component.yaml')
    default_preprocessor_config = os.path.join(pkg_share_dir, 'config', 'preprocessor_component_laundry.yaml')

    # Launch Configurations
    config_file = LaunchConfiguration('config_file')
    preprocessor_config = LaunchConfiguration('preprocessor_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Declare Arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Full path to the laundry detection parameters file'
    )
    preprocessor_config_arg = DeclareLaunchArgument(
        'preprocessor_config',
        default_value=default_preprocessor_config,
        description='Full path to the preprocessor parameters file'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the detection nodes'
    )

    container = ComposableNodeContainer(
        name='laundry_detection_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::PreProcessorComponent',
                name='preprocessor_laundry',
                namespace=namespace,
                parameters=[preprocessor_config, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::LaundryDetectionComponent',
                name='laundry_detection',
                namespace=namespace,
                parameters=[config_file, {'use_sim_time': use_sim_time}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        config_file_arg,
        preprocessor_config_arg,
        use_sim_time_arg,
        namespace_arg,
        container
    ])
