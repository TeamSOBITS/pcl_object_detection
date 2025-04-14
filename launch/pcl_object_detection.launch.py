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

    common_param_ = os.path.join(
        pcl_object_detection_pkg,
        "param",
        "object_common_param.yaml"
    )

    table_param_ = os.path.join(
        pcl_object_detection_pkg,
        "param",
        "object_table_param.yaml"
    )

    floor_param_ = os.path.join(
        pcl_object_detection_pkg,
        "param",
        "object_floor_param.yaml"
    )

    shelf_param_ = os.path.join(
        pcl_object_detection_pkg,
        "param",
        "object_shelf_param.yaml"
    )

    placeable_param_ = os.path.join(
        pcl_object_detection_pkg,
        "param",
        "placeable_param.yaml"
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
        output="screen"
    )

    return LaunchDescription([
        initial_mode_cmd,
        qos_profile_cmd,
        use_rviz_cmd,
        namespace_cmd,
        pcl_node_cmd,
        rviz_node_cmd,
    ])
