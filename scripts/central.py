#!/usr/bin/env python

import traceback

import numpy as np
import rospy
from numpy.typing import NDArray
from std_msgs.msg import Float32MultiArray

from coverage_control.field_generator import FieldGenerator
from coverage_control.utils import ndarray_to_multiarray


class Central:
    """重要度分布を管理する集中制御器"""

    def __init__(self) -> None:
        rospy.init_node("central")

        # fieldを規定するパラメータを取得
        field_param = rospy.get_param("/field")
        self.dim = len(field_param["grid_accuracy"])
        grid_accuracy: NDArray = np.array(field_param["grid_accuracy"])
        limit: NDArray = np.array(field_param["limit"])

        field_generator = FieldGenerator(grid_accuracy=grid_accuracy, limit=limit)
        self.grid_map = field_generator.generate_grid_map()

        timer_period = float(rospy.get_param("/timer_period", default=0.1))

        # pub
        self.phi_pub = rospy.Publisher("/phi", Float32MultiArray, queue_size=1)

        # timer
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

    def timer_callback(self, timer: rospy.Timer) -> None:
        """重要度分布をpublish"""

        # gaussian density function
        phi = np.exp(-10 * (sum([(self.grid_map[i] - 0.5) ** 2 for i in range(self.dim)])) ** 2)
        self.phi_pub.publish(ndarray_to_multiarray(Float32MultiArray, phi))


def main() -> None:
    try:
        Central()
        rospy.spin()
    except:
        rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    main()
