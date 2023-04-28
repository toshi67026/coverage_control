#!/usr/bin/env python

import traceback

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from coverage_control.field_generator import FieldGenerator
from coverage_control.utils import ndarray_to_multiarray


class Central(Node):
    """重要度分布を管理する集中制御器"""

    def __init__(self) -> None:
        super().__init__("central")

        # declare parameter
        self.declare_parameter(
            "grid_accuracy", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        )
        self.declare_parameter("x_limit", [-1.0, 1.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter("y_limit", [-1.0, 1.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter("z_limit", [-1.0, 1.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter(
            "timer_period", 0.1, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )

        # get parameter
        grid_accuracy = np.array(self.get_parameter("grid_accuracy").value)
        self.dim = len(self.get_parameter("grid_accuracy").value)
        limit = np.array(
            [
                self.get_parameter("x_limit").value,
                self.get_parameter("y_limit").value,
                self.get_parameter("z_limit").value,
            ]
        )

        field_generator = FieldGenerator(grid_accuracy=grid_accuracy, limit=limit)
        self.grid_map = field_generator.generate_grid_map()

        timer_period = self.get_parameter("timer_period").get_parameter_value().double_value

        # pub
        self.phi_pub = self.create_publisher(Float32MultiArray, "/phi", 10)

        # timer
        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self) -> None:
        """重要度分布をpublish"""

        # gaussian density function
        phi = np.exp(-10 * (sum([(self.grid_map[i] - 0.5) ** 2 for i in range(self.dim)])) ** 2)
        self.phi_pub.publish(ndarray_to_multiarray(Float32MultiArray, phi))


def main() -> None:
    rclpy.init()
    central = Central()

    try:
        rclpy.spin(central)
    except:
        central.get_logger().error(traceback.format_exc())
    finally:
        central.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
