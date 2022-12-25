#!/usr/bin/env python

import traceback
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from numpy.typing import NDArray
from std_msgs.msg import ColorRGBA, Float32MultiArray, Header
from visualization_msgs.msg import Marker

from coverage_control.field_generator import FieldGenerator
from coverage_control.utils import multiarray_to_ndarray, padding


class PhiMarkerVisualizer:
    """Markerを用いて重要度分布を可視化

    Note:
        grid_mapは以下の並びとなるよう，reshapeと転置により整形
        self.rows = [
            [x1, y1, ...],
            [x2, y2, ...]
                :
            [xn, yn, ...]]
    """

    def __init__(self) -> None:
        rospy.init_node("phi_marker_visualizer")

        world_frame = str(rospy.get_param("/world_frame", default="world"))

        # fieldを規定するパラメータを取得
        field_param = rospy.get_param("/field")
        grid_accuracy: NDArray = np.array(field_param["grid_accuracy"])
        limit: NDArray = np.array(field_param["limit"])
        dim = len(grid_accuracy)

        # 次元に応じて適切な透過度を選択
        self.alpha = 0.7 - 0.15 * dim

        field_generator = FieldGenerator(grid_accuracy=grid_accuracy, limit=limit)
        grid_map = field_generator.generate_grid_map()

        self.rows: NDArray = np.array(grid_map).reshape([dim, -1]).T

        # 重要度分布描画用のMarkerを作成
        self.phi_marker = Marker(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id=world_frame,
            ),
            ns="phi_marker",
            action=Marker.ADD,
            type=Marker.CUBE_LIST,
            pose=Pose(orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            scale=Vector3(
                *padding(
                    original_array=field_generator.grid_span,
                    padding_value=0.05,
                )
            ),
        )

        # pub
        self.phi_marker_pub = rospy.Publisher("/phi_marker", Marker, queue_size=1)

        # sub
        rospy.Subscriber("/phi", Float32MultiArray, self.phi_callback)

    def phi_callback(self, msg: Float32MultiArray) -> None:
        phi = multiarray_to_ndarray(float, np.float32, msg).reshape([-1, 1])

        # 次元を調整しつつmatplotlibのcolor_mapを利用して，重要度を色で表現
        rgba_phi: NDArray = plt.get_cmap("jet")(phi).squeeze()

        self.phi_marker.header.stamp = rospy.Time.now()
        # 2次元以下の場合は足りない座標分を0埋め
        self.phi_marker.points = [Point(*point) for point in [padding(point) for point in self.rows]]
        self.phi_marker.colors = [ColorRGBA(*[*rgb, self.alpha]) for rgb in rgba_phi[:, 0:3]]
        self.phi_marker_pub.publish(self.phi_marker)


def main() -> None:
    try:
        PhiMarkerVisualizer()
        rospy.spin()
    except:
        rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    main()
