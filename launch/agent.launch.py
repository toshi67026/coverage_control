#!/usr/bin/env python3

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetParametersFromFile


def launch_setup(context: LaunchContext) -> List[GroupAction]:
    pkg_coverage_control = get_package_share_directory("coverage_control")
    agent_config = LaunchConfiguration(
        "agent_config", default=os.path.join(pkg_coverage_control, "config", "agent.params.yaml")
    )
    base_config = LaunchConfiguration(
        "base_config", default=os.path.join(pkg_coverage_control, "config", "base.params.yaml")
    )

    # performでLaunchConfigurationの中身を取得
    agent_id_str = LaunchConfiguration("agent_id").perform(context)
    agent_name = "agent" + agent_id_str

    agent_group = GroupAction(
        actions=[
            PushRosNamespace(agent_name),
            SetParameter(name="agent_id", value=agent_id_str),
            SetParametersFromFile(base_config),
            SetParametersFromFile(agent_config),
            Node(
                package="coverage_control",
                executable="agent_body",
            ),
            Node(
                package="coverage_control",
                executable="controller",
            ),
            Node(
                package="coverage_control",
                executable="sensing_region_marker_visualizer",
            ),
            Node(
                package="coverage_control",
                executable="sensing_region_pointcloud_visualizer",
            ),
        ],
    )
    return [agent_group]


def generate_launch_description() -> LaunchDescription:
    DeclareLaunchArgument("agent_id")

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
