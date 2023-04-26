#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParametersFromFile


def generate_launch_description() -> LaunchDescription:
    pkg_coverage_control = get_package_share_directory("coverage_control")
    rviz_config = LaunchConfiguration("rviz_config", default=os.path.join(pkg_coverage_control, "rviz", "2d.rviz"))
    base_config = LaunchConfiguration("base_config", default=os.path.join(pkg_coverage_control, "config", "base.params.yaml"))

    # TODO(toshi) launch argumentのdimに応じてagent_numやfield paramを変更
    dim = LaunchConfiguration("dim", default="2")
    dim_arg = DeclareLaunchArgument("dim", default_value="2")

    agent_num = 3

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    # group actionでまとめることでconfigを共通で与える
    central_group = GroupAction(
        actions=[
            SetParametersFromFile(base_config),
            Node(
                package="coverage_control",
                executable="pose_collector",
                output="screen",
            ),
            Node(
                package="coverage_control",
                executable="central",
            ),
            Node(
                package="coverage_control",
                executable="phi_marker_visualizer",
            ),
            Node(
                package="coverage_control",
                executable="phi_pointcloud_visualizer",
            ),
        ]
    )

    agent_launch_list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_coverage_control, "launch", "agent.launch.py")]),
            launch_arguments={"agent_id": str(agent_id)}.items(),
        )
        for agent_id in range(agent_num)
    ]

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(dim_arg)

    ld.add_action(rviz_node)
    ld.add_action(central_group)
    [ld.add_action(agent_launch) for agent_launch in agent_launch_list]

    return ld
