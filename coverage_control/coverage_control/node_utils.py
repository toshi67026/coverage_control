#!/usr/bin/env python

from dataclasses import dataclass
from typing import Tuple

import numpy as np
from numpy.typing import NDArray
from rclpy.node import Node


AXIS_KEYS: Tuple[str, ...] = ("x_limit", "y_limit", "z_limit")


@dataclass(frozen=True)
class FieldParams:
    grid_accuracy: NDArray[np.int_]
    limit: NDArray[np.float64]
    dim: int


def get_field_params(node: Node) -> FieldParams:
    grid_accuracy = np.array(node.get_parameter("grid_accuracy").value, dtype=int)
    dim = int(grid_accuracy.size)
    if dim not in (1, 2, 3):
        raise ValueError(f"grid_accuracy must have 1-3 dimensions, got {dim}")
    if np.any(grid_accuracy <= 0):
        raise ValueError(f"grid_accuracy must be positive: {grid_accuracy.tolist()}")

    limit = np.array([node.get_parameter(key).value for key in AXIS_KEYS], dtype=float)
    return FieldParams(grid_accuracy=grid_accuracy, limit=limit, dim=dim)

