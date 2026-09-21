# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def parse_yaml(context):
    crazyflies_yaml = LaunchConfiguration("crazyflies_yaml_file").perform(context)
    with open(crazyflies_yaml, "r") as file:
        crazyflies = yaml.safe_load(file)

    # store the fileversion
    fileversion = 1
    if "fileversion" in crazyflies:
        fileversion = crazyflies["fileversion"]

    # server params
    server_yaml = os.path.join(
        get_package_share_directory("crazyflie"), "config", "server.yaml"
    )

    with open(server_yaml, "r") as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)

    server_params = [crazyflies] + [
        server_yaml_content["/crazyflie_server"]["ros__parameters"]
    ]

    # robot description
    urdf = os.path.join(
        get_package_share_directory("crazyflie"), "urdf", "crazyflie_description.urdf"
    )

    with open(urdf, "r") as f:
        robot_desc = f.read()

    server_params[1]["robot_description"] = robot_desc

    # construct motion_capture_configuration
    motion_capture_yaml = LaunchConfiguration("motion_capture_yaml_file").perform(
        context
    )
    with open(motion_capture_yaml, "r") as ymlfile:
        motion_capture_content = yaml.safe_load(ymlfile)

    motion_capture_params = motion_capture_content["/motion_capture_tracking"][
        "ros__parameters"
    ]
    motion_capture_params["rigid_bodies"] = dict()
    for key, value in crazyflies["robots"].items():
        type = crazyflies["robot_types"][value["type"]]
        if value["enabled"] and (
            (fileversion == 1 and type["motion_capture"]["enabled"])
            or (
                fileversion >= 2
                and type["motion_capture"]["tracking"] == "librigidbodytracker"
            )
        ):
            motion_capture_params["rigid_bodies"][key] = {
                "initial_position": value["initial_position"],
                "marker": type["motion_capture"]["marker"],
                "dynamics": type["motion_capture"]["dynamics"],
            }

    # copy relevant settings to server params
    server_params[1]["poses_qos_deadline"] = motion_capture_params["topics"]["poses"][
        "qos"
    ]["deadline"]

    crazysim = Node(
        package="crazyflie_sim",
        executable="crazyflie_server",
        name="crazyflie_server",
        output="screen",
        emulate_tty=True,
        parameters=server_params,
    )
    return [crazysim]


def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory("sala4_bringup")
    pkg_project_gazebo = get_package_share_directory("ros_gz_crazyflie_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    default_crazyflies_yaml_path = os.path.join(
        get_package_share_directory("crazyflie"), "config", "crazyflies.yaml"
    )
    default_motion_capture_yaml_path = os.path.join(
        get_package_share_directory("crazyflie"), "config", "motion_capture.yaml"
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("gazebo_launch")),
        launch_arguments={
            "gz_args": PathJoinSubstitution(
                [pkg_project_gazebo, "worlds", "crazyflie_world.sdf -r"]
            )
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup, "config", "ros_gz_crazyflie_bridge.yaml"
                ),
            }
        ],
        output="screen",
    )

    gz_odom_logger = Node(
        package="sala4_bringup",
        executable="gz_odom_logger",
        name="gz_odom_logger",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}],
    )

    # control = Node(
    #     package="ros_gz_crazyflie_control",
    #     executable="control_services",
    #     output="screen",
    #     parameters=[
    #         {"hover_height": 1.0},
    #         {"robot_prefix": "/crazyflie"},
    #         {"incoming_twist_topic": "/gazebo/command/twist"},
    #         {"max_ang_z_rate": 0.4},
    #     ],
    # )

    # interpreter = Node(
    #     package="cf_fullstate_to_twist",
    #     executable="fullstate_to_twist",
    #     output="screen",
    # )

    # rviz_config_path = os.path.join(
    #     get_package_share_directory("sala4_bringup"),
    #     "config",
    #     "sim_mapping.rviz",
    # )

    # rviz = Node(
    #     package="rviz2",
    #     namespace="",
    #     executable="rviz2",
    #     name="rviz2",
    #     arguments=["-d", rviz_config_path],
    #     parameters=[{"use_sim_time": True}],
    # )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_launch", default_value="True"),
            DeclareLaunchArgument(
                "crazyflies_yaml_file", default_value=default_crazyflies_yaml_path
            ),
            DeclareLaunchArgument(
                "motion_capture_yaml_file",
                default_value=default_motion_capture_yaml_path,
            ),
            OpaqueFunction(function=parse_yaml),
            gz_sim,
            bridge,
            gz_odom_logger,
            # control,
            # interpreter,
            # rviz,
        ]
    )