#!/usr/bin/env python

import traceback
from typing import List, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseArray, Twist, Vector3
from numpy.typing import NDArray
from std_msgs.msg import Float32MultiArray, Int8MultiArray

from coverage_control.field_generator import FieldGenerator
from coverage_control.utils import multiarray_to_ndarray, ndarray_to_multiarray, padding
from coverage_control.voronoi import Voronoi


class Controller:
    """エージェントのローカル制御器"""

    def __init__(self) -> None:
        rospy.init_node("controller")

        self.world_frame = str(rospy.get_param("/world_frame", default="world"))

        self.agent_id = int(rospy.get_param("agent_id", default=0))
        self.agent_num = int(rospy.get_param("/agent_num", default=1))

        # fieldを規定するパラメータを取得
        field_param = rospy.get_param("/field")
        grid_accuracy: NDArray = np.array(field_param["grid_accuracy"])
        limit: NDArray = np.array(field_param["limit"])
        self.dim = len(grid_accuracy)

        field_generator = FieldGenerator(grid_accuracy=grid_accuracy, limit=limit)

        # 重要度分布初期化
        self.phi = field_generator.generate_phi()
        self.grid_map = field_generator.generate_grid_map()
        self.point_density = np.prod(field_generator.grid_span)
        self.voronoi = Voronoi()

        self.ref_pose = Pose()
        self.curr_pose = Pose()
        self.kp = float(rospy.get_param("/kp", default=1))
        timer_period = float(rospy.get_param("/timer_period", default=0.1))

        # pub
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.sensing_region_pub = rospy.Publisher("sensing_region", Int8MultiArray, queue_size=1)

        # sub
        rospy.Subscriber("curr_pose", Pose, self.curr_pose_callback, queue_size=1)
        rospy.Subscriber("/curr_pose_array", PoseArray, self.curr_pose_array_callback)
        rospy.Subscriber("/phi", Float32MultiArray, self.phi_callback)

        # timer
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

    def curr_pose_callback(self, msg: Pose) -> None:
        self.curr_pose = msg

    def curr_pose_array_callback(self, msg: PoseArray) -> None:
        """センシング領域とその重心を計算して目標座標へ反映

        Note:
            センシング領域のpublishに際しては，以下の並びになるようreshapeと転置により整形
            [[x1, y1, ...],
            [x2, y2, ...]
                :
            [xn, yn, ...]]
        """

        # 近隣エージェントの位置を用いてセンシング領域を計算
        centroid_position, sensing_region_grid_points, sensing_region = self.calc_voronoi_tesselation(msg.poses)

        # 各次元に対応して使わない部分を0で埋めた上で，unpackしたもの目標座標とする
        self.ref_pose = Pose(position=Point(*padding(centroid_position)))

        # センシング領域をpublish
        self.sensing_region_pub.publish(ndarray_to_multiarray(Int8MultiArray, sensing_region))

    def calc_voronoi_tesselation(self, pose_list: List[Pose]) -> Tuple[NDArray, List[NDArray], NDArray]:
        all_agent_position_list: List[NDArray] = []
        # Pose型のリストから各エージェント位置を抜き出してNDArray形式へ変換，格納
        for pose in pose_list:
            all_agent_position_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))

        agent_position = all_agent_position_list[self.agent_id]

        # 自分の位置のみ除外
        neighbor_agent_position_list = [
            neighbor_agent_position
            for agent_id, neighbor_agent_position in enumerate(all_agent_position_list)
            if agent_id != self.agent_id
        ]

        # センシング領域を計算
        # 円形センサーモデル使用時は，性能関数としてK.Sugimoto et al. 2015のh_{1}を採用した場合の最適解に相当
        centroid_position, sensing_region_grid_points, sensing_region = self.voronoi.calc_tesselation(
            agent_position=agent_position,
            neighbor_agent_position_list=neighbor_agent_position_list,
            phi=self.phi,
            grid_map=self.grid_map,
            point_density=self.point_density,
        )
        return centroid_position, sensing_region_grid_points, sensing_region

    def phi_callback(self, msg: Float32MultiArray) -> None:
        self.phi = multiarray_to_ndarray(float, np.float32, msg)

    def timer_callback(self, timer: rospy.Timer) -> None:
        # 位置偏差
        x_err = self.ref_pose.position.x - self.curr_pose.position.x
        y_err = self.ref_pose.position.y - self.curr_pose.position.y
        z_err = self.ref_pose.position.z - self.curr_pose.position.z

        cmd_vel = Twist(
            linear=Vector3(
                x=self.kp * x_err,
                y=self.kp * y_err,
                z=self.kp * z_err,
            )
        )
        self.cmd_vel_pub.publish(cmd_vel)


def main() -> None:
    try:
        Controller()
        rospy.spin()
    except:
        rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    main()
