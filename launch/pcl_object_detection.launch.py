import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pcl_object_detection_pkg = get_package_share_directory("pcl_object_detection")

    initial_mode = LaunchConfiguration("initial_mode")
    initial_mode_cmd = DeclareLaunchArgument(
        "initial_mode", default_value="0",
        description="initial mode choise of [0:OFF, 1:Table, 2:Floor, 3:Shelf, 4:Placeable]"
    )

    qos_profile = LaunchConfiguration("qos_profile")
    qos_profile_cmd = DeclareLaunchArgument(
        "qos_profile", default_value="BEST_EFFORT",
        description="QoS profile choise of [RELIABLE, BEST_EFFORT]"
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True",
        description="Use rviz bringup"
    )

    filter_vertical_structures = LaunchConfiguration("filter_vertical_structures")
    filter_vertical_structures_cmd = DeclareLaunchArgument(
        "filter_vertical_structures", default_value="False",
        description="Enable vertical structure filtering for floor mode"
    )

    common_param_yaml = os.path.join(
        pcl_object_detection_pkg,
        "param",
        "object_common_param.yaml"
    )

    table_param_yaml = os.path.join(
        pcl_object_detection_pkg,
        "param",
        "object_table_param.yaml"
    )

    floor_param_yaml = os.path.join(
        pcl_object_detection_pkg,
        "param",
        "object_floor_param.yaml"
    )

    shelf_param_yaml = os.path.join(
        pcl_object_detection_pkg,
        "param",
        "object_shelf_param.yaml"
    )

    placeable_param_yaml = os.path.join(
        pcl_object_detection_pkg,
        "param",
        "placeable_param.yaml"
    )

    common_param_ = LaunchConfiguration("common_param")
    common_param_cmd = DeclareLaunchArgument(
        "common_param", default_value=common_param_yaml,
        description="Path to common parameter yaml file"
    )

    table_param_ = LaunchConfiguration("table_param")
    table_param_cmd = DeclareLaunchArgument(
        "table_param", default_value=table_param_yaml,
        description="Path to table parameter yaml file"
    )

    floor_param_ = LaunchConfiguration("floor_param")
    floor_param_cmd = DeclareLaunchArgument(
        "floor_param", default_value=floor_param_yaml,
        description="Path to floor parameter yaml file"
    )

    shelf_param_ = LaunchConfiguration("shelf_param")
    shelf_param_cmd = DeclareLaunchArgument(
        "shelf_param", default_value=shelf_param_yaml,
        description="Path to shelf parameter yaml file"
    )

    placeable_param_ = LaunchConfiguration("placeable_param")
    placeable_param_cmd = DeclareLaunchArgument(
        "placeable_param", default_value=placeable_param_yaml,
        description="Path to placeable parameter yaml file"
    )

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="pcl_object_detection", description="Namespace for the nodes"
    )

    pcl_node_cmd = Node(
        package="pcl_object_detection",
        executable="pcl_object_detection",
        name="pcl_object_detection",
        namespace=namespace,
        parameters=[
            {
                "initial_mode": initial_mode,
                "qos_profile" : qos_profile,
                "floor.filter_vertical_structures": filter_vertical_structures,
            },
            common_param_,
            table_param_,
            floor_param_,
            shelf_param_,
            placeable_param_,
        ],
        output="screen"
    )

    rviz_node_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="pcl_rviz_node",
        arguments=['-d', os.path.join(pcl_object_detection_pkg, "rviz", "pcl_object_detection.rviz")],
        output="screen",
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        initial_mode_cmd,
        qos_profile_cmd,
        use_rviz_cmd,
        filter_vertical_structures_cmd,
        common_param_cmd,
        table_param_cmd,
        floor_param_cmd,
        shelf_param_cmd,
        placeable_param_cmd,
        namespace_cmd,
        pcl_node_cmd,
        rviz_node_cmd,
    ])
