#!/usr/bin/env python

import traceback

import matplotlib.colors as mcolors
import numpy as np
import rospy
from coverage_control.field_generator import FieldGenerator
from coverage_control.utils import color_list, multiarray_to_ndarray
from numpy.typing import NDArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Int8MultiArray


class SensingRegionPointCloudVisualizer:
    """PointCloud2を用いてセンシング領域を可視化"""

    def __init__(self) -> None:
        rospy.init_node("sensing_region_pointcloud_visualizer")

        world_frame = str(rospy.get_param("/world_frame", default="world"))

        self.agent_id = int(rospy.get_param("agent_id", default=0))

        # fieldを規定するパラメータを取得
        field_param = rospy.get_param("/field")
        grid_accuracy: NDArray = np.array(field_param["grid_accuracy"])
        limit: NDArray = np.array(field_param["limit"])
        field_generator = FieldGenerator(grid_accuracy=grid_accuracy, limit=limit)
        self.grid_map = field_generator.generate_grid_map()
        self.dim = len(grid_accuracy)

        # センシング領域描画用のpointcloudを作成
        self.sensing_region_pointcloud = PointCloud2(
            header=Header(
                stamp=rospy.Time.now(),
                frame_id=world_frame,
            ),
            height=1,
            fields=[
                PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
                PointField("r", 12, PointField.FLOAT32, 1),
                PointField("g", 16, PointField.FLOAT32, 1),
                PointField("b", 20, PointField.FLOAT32, 1),
            ],
            is_bigendian=False,
            point_step=24,
            is_dense=True,
        )

        # pub
        self.sensing_region_pointcloud_pub = rospy.Publisher("sensing_region_pointcloud", PointCloud2, queue_size=1)

        # sub
        rospy.Subscriber("sensing_region", Int8MultiArray, self.sensing_region_callback)

    def sensing_region_callback(self, msg: Int8MultiArray) -> None:
        """Int8Multiarrayからpointcloudを生成してpublish

        Args:
            msg (Int8MultiArray): センシング領域

        Note:
            センシング領域の格子点計算に際してはpointcloud生成用にreshapeと転置により整形

            points = [
            [x1, y1, z1, r1, g1, b1],
            [x2, y2, z2, r2, g2, b2],
                    :
            [xn, yn, zn, rn, gn, bn]]
        """
        sensing_region = multiarray_to_ndarray(bool, np.bool_, msg)
        sensing_region_grid_map = (
            np.array([self.grid_map[i][sensing_region] for i in range(self.dim)]).reshape(self.dim, -1).T
        )

        # np.repeatのために2次元配列として作成
        rgba_agent: NDArray = np.array([mcolors.to_rgba(color_list[self.agent_id])])
        assert rgba_agent.shape == (1, 4)

        # 2次元以下の場合は足りない座標分を0埋め
        points = np.hstack(
            [
                sensing_region_grid_map,
                np.zeros([sensing_region_grid_map.shape[0], 3 - self.dim]),
                np.repeat(rgba_agent[:, 0:3], sensing_region_grid_map.shape[0], axis=0),
            ]
        ).astype(np.float32)

        num_points = len(points)
        self.sensing_region_pointcloud.width = num_points
        self.sensing_region_pointcloud.row_step = self.sensing_region_pointcloud.point_step * num_points
        self.sensing_region_pointcloud.data = points.tobytes()
        self.sensing_region_pointcloud_pub.publish(self.sensing_region_pointcloud)


def main() -> None:
    try:
        SensingRegionPointCloudVisualizer()
        rospy.spin()
    except:
        rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    main()
