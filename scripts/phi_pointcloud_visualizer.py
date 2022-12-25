#!/usr/bin/env python

import traceback
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import rospy
from numpy.typing import NDArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header

from coverage_control.field_generator import FieldGenerator
from coverage_control.utils import padding


class PhiPointCloudVisualizer:
    """PointCloud2を用いて重要度分布を可視化

    Note:
        grid_mapは以下の並びとなるよう，reshapeと転置により整形
        self.rows = [
            [x1, y1, ...],
            [x2, y2, ...]
                :
            [xn, yn, ...]]
    """

    def __init__(self):
        rospy.init_node("phi_pointcloud_visualizer")

        world_frame = str(rospy.get_param("/world_frame", default="world"))

        # fieldを規定するパラメータを取得
        field_param = rospy.get_param("/field")
        grid_accuracy: NDArray = np.array(field_param["grid_accuracy"])
        limit: NDArray = np.array(field_param["limit"])
        self.dim = len(grid_accuracy)

        field_generator = FieldGenerator(grid_accuracy=grid_accuracy, limit=limit)
        grid_map = field_generator.generate_grid_map()
        self.rows: NDArray = np.array(grid_map).reshape([self.dim, -1]).T

        # 重要度分布描画用のpointcloudを作成
        self.phi_pointcloud = PointCloud2(
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
            is_dense=12,
        )

        # pub
        self.phi_pointcloud_pub = rospy.Publisher("phi_pointcloud", PointCloud2, queue_size=1)

        # sub
        rospy.Subscriber("phi", Float32MultiArray, self.phi_callback)

    def phi_callback(self, msg: Float32MultiArray) -> None:
        self.publish_pointcloud(msg.data)

    def publish_pointcloud(self, data: List[float]) -> None:
        """Float32Multiarrayからpointcloudを生成してpublish

        Args:
            data (List[float]): 重要度分布

        Note:
            points = [
            [x1, y1, z1, r1, g1, b1],
            [x2, y2, z2, r2, g2, b2],
                    :
            [xn, yn, zn, rn, gn, bn]]

            pointsのz座標等にphiを反映することで，座標でも重要度を表現可能
        """
        phi = np.array(data).reshape([-1, 1])

        # 次元を調整しつつmatplotlibのcolor_mapを利用して，重要度を色で表現
        rgba_phi: NDArray = plt.get_cmap("jet")(phi).squeeze()

        # 2次元以下の場合は足りない座標分を0埋め
        points = np.hstack(
            [
                self.rows,
                np.zeros([self.rows.shape[0], 3 - self.dim]),
                rgba_phi[:, 0:3],
            ]
        ).astype(np.float32)

        num_points = len(points)
        self.phi_pointcloud.width = num_points
        self.phi_pointcloud.row_step = self.phi_pointcloud.point_step * num_points
        self.phi_pointcloud.data = points.tobytes()
        self.phi_pointcloud_pub.publish(self.phi_pointcloud)


def main() -> None:
    try:
        PhiPointCloudVisualizer()
        rospy.spin()
    except:
        rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    main()
