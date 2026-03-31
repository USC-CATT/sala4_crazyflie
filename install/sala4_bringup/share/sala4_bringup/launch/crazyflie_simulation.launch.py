import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_crazyflie_gazebo = get_package_share_directory(
        "ros_gz_crazyflie_bringup"
    )

    # Setup to launch a crazyflie gazebo simulation from the ros_gz_crazyflie project
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_project_crazyflie_gazebo, "launch", "crazyflie_simulation.launch.py"
            )
        )
    )
    
    rviz_config_path = os.path.join(
        get_package_share_directory("sala4_bringup"),
        "config",
        "sim_mapping.rviz",
    )

    rviz = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([crazyflie_simulation, rviz])
