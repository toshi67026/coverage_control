#!/usr/bin/env python

import traceback

import numpy as np
import rospy
from coverage_control.field_generator import FieldGenerator
from coverage_control.utils import color_list, get_color_rgba, multiarray_to_ndarray, padding
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from numpy.typing import NDArray
from std_msgs.msg import Header, Int8MultiArray
from visualization_msgs.msg import Marker


class SensingRegionMarkerVisualizer:
    """Markerを用いてセンシング領域を可視化"""

    def __init__(self) -> None:
        rospy.init_node("sensing_region_marker_visualizer")

        world_frame = str(rospy.get_param("/world_frame", default="world"))

        self.agent_id = int(rospy.get_param("agent_id", default=0))

        # fieldを規定するパラメータを取得
        field_param = rospy.get_param("/field")
        grid_accuracy: NDArray = np.array(field_param["grid_accuracy"])
        limit: NDArray = np.array(field_param["limit"])
        field_generator = FieldGenerator(grid_accuracy=grid_accuracy, limit=limit)
        self.grid_map = field_generator.generate_grid_map()
        self.dim = len(grid_accuracy)
        # 次元に応じて適切な透過度を選択
        self.alpha = 0.7 - 0.15 * self.dim

        # センシング領域描画用のMarkerを作成
        self.sensing_region_marker = Marker(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id=world_frame,
            ),
            ns="sensing_region_marker",
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
        self.sensing_region_marker_pub = rospy.Publisher("sensing_region_marker", Marker, queue_size=1)

        # sub
        rospy.Subscriber("sensing_region", Int8MultiArray, self.sensing_region_callback)

    def sensing_region_callback(self, msg: Int8MultiArray) -> None:
        """Int8Multiarrayからpointcloudを生成してpublish

        Args:
            msg (Int8MultiArray): センシング領域

        Note:
            センシング領域の格子点計算に際しては，以下の並びになるようreshapeと転置により整形
            [[x1, y1, ...],
            [x2, y2, ...]
                :
            [xn, yn, ...]]
        """
        sensing_region = multiarray_to_ndarray(bool, np.bool_, msg)
        sensing_region_grid_map = (
            np.array([self.grid_map[i][sensing_region] for i in range(self.dim)]).reshape(self.dim, -1).T
        )

        self.sensing_region_marker.header.stamp = rospy.Time.now()
        # 2次元以下の場合は足りない座標分を0埋め
        self.sensing_region_marker.points = [
            Point(*point) for point in [padding(point) for point in sensing_region_grid_map]
        ]
        self.sensing_region_marker.colors = [
            get_color_rgba(color_list[self.agent_id], self.alpha)
        ] * sensing_region_grid_map.shape[0]

        self.sensing_region_marker_pub.publish(self.sensing_region_marker)


def main() -> None:
    try:
        SensingRegionMarkerVisualizer()
        rospy.spin()
    except:
        rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    main()
