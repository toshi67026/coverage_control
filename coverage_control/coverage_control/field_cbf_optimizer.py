#!/usr/bin/env python3

import traceback
from typing import List, Tuple

import numpy as np
import rclpy
from cbfpy.cbf import Pnorm2dCBF
from cbfpy.cbf_qp_solver import CBFNomQPSolver
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

from .coverage_utils.utils import get_color_rgba


class CBFOptimizer:
    def __init__(self) -> None:
        self.qp_nom_solver = CBFNomQPSolver()
        self.P = np.eye(2)

        self.pnorm2d_cbf = Pnorm2dCBF()

        # initialize(must be overwritten)
        self.set_parameters(np.zeros(2), np.ones(2))

    def set_parameters(
        self, center: np.ndarray, width: np.ndarray, theta: float = 0.0, p: float = 2.0, keep_inside: bool = True
    ) -> None:
        self.pnorm2d_cbf.set_parameters(center, width, theta, p, keep_inside)

    def get_parameters(self) -> Tuple[np.ndarray, np.ndarray, float, float, bool]:
        return self.pnorm2d_cbf.get_parameters()

    def _calc_constraints(self, agent_position: np.ndarray) -> None:
        self.pnorm2d_cbf.calc_constraints(agent_position)

    def _get_constraints(self) -> Tuple[List[np.ndarray], List[float]]:
        G, alpha_h = self.pnorm2d_cbf.get_constraints()
        return [G], [alpha_h]

    def optimize(self, nominal_input: np.ndarray, agent_position: np.ndarray) -> Tuple[str, np.ndarray]:
        self._calc_constraints(agent_position)
        G_list, alpha_h_list = self._get_constraints()

        try:
            return self.qp_nom_solver.optimize(nominal_input, self.P, G_list, alpha_h_list)
        except Exception as e:
            raise e


class FieldCBFOptimizer(Node):
    def __init__(self) -> None:
        super().__init__("field_cbf_optimizer")

        # declare parameter
        self.declare_parameter(
            "world_frame", "world", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter("activate_cbf", True, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter(
            "cbf_x_limit", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter(
            "cbf_y_limit", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter("cbf_theta", 0.0, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            "cbf_keep_inside", True, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)
        )

        # get parameter
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.activate_cbf = bool(self.get_parameter("activate_cbf").value)
        limit = np.array(
            [
                self.get_parameter("cbf_x_limit").value,
                self.get_parameter("cbf_y_limit").value,
            ]
        )
        dim = len(limit)
        self.theta = float(self.get_parameter("cbf_theta").value)
        self.keep_inside = bool(self.get_parameter("cbf_keep_inside").value)

        self.optimizer = CBFOptimizer()

        self.center = np.array([sum(limit[i]) / dim for i in range(dim)])
        self.width = np.array([(limit[i][1] - limit[i][0]) / 2.0 for i in range(dim)])

        self.curr_pose = Pose()

        orientation_array = quaternion_from_euler(ai=0, aj=0, ak=self.theta)
        # 障害物を表示
        self.obstacle_marker = Marker(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame),
            ns="obstacle_marker",
            action=Marker.ADD,
            type=Marker.CYLINDER,
            pose=Pose(
                position=Point(x=self.center[0], y=self.center[1], z=0.0),
                orientation=Quaternion(
                    x=orientation_array[0],
                    y=orientation_array[1],
                    z=orientation_array[2],
                    w=orientation_array[3],
                ),
            ),
            scale=Vector3(x=self.width[0] * 2, y=self.width[1] * 2, z=0.1),
            color=get_color_rgba(color="w", alpha=0.7),
        )

        # pub
        self.cmd_vel_opt_pub = self.create_publisher(Twist, "cmd_vel_opt", 10)
        self.obstacle_marker_pub = self.create_publisher(Marker, "obstacle_marker", 10)

        # sub
        self.create_subscription(Pose, "curr_pose", self.curr_pose_callback, 10)
        self.create_subscription(Twist, "cmd_vel_nom", self.cmd_vel_nom_callback, 10)

    def curr_pose_callback(self, msg: Pose) -> None:
        self.curr_pose = msg

    def cmd_vel_nom_callback(self, msg: Twist) -> None:
        cmd_vel_opt = msg

        # optimize
        if self.activate_cbf:
            self.optimizer.set_parameters(
                center=self.center, width=self.width, theta=self.theta, keep_inside=self.keep_inside
            )
            agent_position = np.array([self.curr_pose.position.x, self.curr_pose.position.y])
            nominal_input = np.array(
                [
                    msg.linear.x,
                    msg.linear.y,
                ]
            )
            # 最適入力を計算
            _, optimal_input = self.optimizer.optimize(nominal_input, agent_position)
            cmd_vel_opt.linear = Vector3(x=float(optimal_input[0]), y=float(optimal_input[1]))

        self.cmd_vel_opt_pub.publish(cmd_vel_opt)

        self.obstacle_marker.header.stamp = self.get_clock().now().to_msg()
        self.obstacle_marker_pub.publish(self.obstacle_marker)


def main() -> None:
    rclpy.init()
    field_cbf_optimizer = FieldCBFOptimizer()

    try:
        rclpy.spin(field_cbf_optimizer)
    except:
        field_cbf_optimizer.get_logger().error(traceback.format_exc())
    finally:
        field_cbf_optimizer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
