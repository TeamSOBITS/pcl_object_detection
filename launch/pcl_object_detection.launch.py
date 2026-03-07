import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_share = get_package_share_directory('pcl_object_detection')

    preprocessor_config = os.path.join(pkg_share, 'config', 'preprocessor_component.yaml')
    table_config = os.path.join(pkg_share, 'config', 'table_detection_component.yaml')
    floor_config = os.path.join(pkg_share, 'config', 'floor_detection_component.yaml')
    shelf_config = os.path.join(pkg_share, 'config', 'shelf_detection_component.yaml')
    placeable_config = os.path.join(pkg_share, 'config', 'placeable_detection_component.yaml')
    line_config = os.path.join(pkg_share, 'config', 'line_detection_component.yaml')

    # The container (Holds everything in one process for zero-copy)
    container = ComposableNodeContainer(
        name='pcl_detection_container',
        namespace='pcl_object_detection',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # PreProcessor (Standard Component, always running)
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::PreProcessorComponent',
                name='preprocessor',
                parameters=[preprocessor_config],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Worker Nodes (Lifecycle Components)
            # Table Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::TableDetectionComponent',
                name='table_detection',
                parameters=[table_config],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Floor Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::FloorDetectionComponent',
                name='floor_detection',
                parameters=[floor_config],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Shelf Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::ShelfDetectionComponent',
                name='shelf_detection',
                parameters=[shelf_config],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Placeable Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::PlaceableDetectionComponent',
                name='placeable_detection',
                parameters=[placeable_config],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Line Detection Worker
            ComposableNode(
                package='pcl_object_detection',
                plugin='pcl_object_detection::LineDetectionComponent',
                name='line_detection',
                parameters=[line_config],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
