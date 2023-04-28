#!/usr/bin/env python3

import os
from typing import List, Union

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParametersFromFile


def launch_setup(context: LaunchContext) -> List[Union[Node, GroupAction, LaunchDescription]]:
    dim = int(LaunchConfiguration("dim").perform(context))
    assert dim in range(1, 4), f"invalid dimension: {dim}"
    agent_num = int(LaunchConfiguration("agent_num").perform(context))
    assert agent_num in range(1, 6), f"invalid agent_num: {agent_num}"

    pkg_coverage_control = get_package_share_directory("coverage_control")
    rviz_config = os.path.join(pkg_coverage_control, "rviz", f"{dim}d.rviz")
    assert os.path.exists(rviz_config)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    base_config = os.path.join(pkg_coverage_control, "config", "base.params.yaml")
    agent_config = os.path.join(pkg_coverage_control, "config", f"{dim}d.params.yaml")

    # group actionでまとめることでconfigを共通で与える
    central_group = GroupAction(
        actions=[
            SetParametersFromFile(agent_config),
            SetParametersFromFile(base_config),
            Node(
                package="coverage_control",
                executable="pose_collector",
                parameters=[{"agent_num": agent_num}],
                output="screen",
            ),
            Node(package="coverage_control", executable="central"),
            Node(package="coverage_control", executable="phi_marker_visualizer"),
            Node(package="coverage_control", executable="phi_pointcloud_visualizer"),
        ]
    )

    agent_launch_list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_coverage_control, "launch", "agent.launch.py")]),
            launch_arguments={
                "base_config": base_config,
                "agent_config": agent_config,
                "agent_id": str(agent_id),
            }.items(),
        )
        for agent_id in range(agent_num)
    ]
    # nodeの起動順に起因するagentのジャンプを防ぐため，central系を後に
    return agent_launch_list + [rviz_node, central_group]


def generate_launch_description() -> LaunchDescription:
    DeclareLaunchArgument("dim", default_value="2", description="\in {1, 2, 3}")
    DeclareLaunchArgument("agent_num", default_value="3", description="< 7")

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
