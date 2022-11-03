#!/usr/bin/env python

import numpy as np
from std_msgs.msg import ColorRGBA

# 各種描画に使う色を定義
color_list = ["r", "b", "y", "m", "c", "g"]


def get_color_rgba(color_initial: str, alpha: float = 1.0) -> ColorRGBA:
    if color_initial == "r":
        return ColorRGBA(r=1.0, a=alpha)
    elif color_initial == "g":
        return ColorRGBA(g=1.0, a=alpha)
    elif color_initial == "b":
        return ColorRGBA(b=1.0, a=alpha)
    elif color_initial == "m":
        return ColorRGBA(r=1.0, b=1.0, a=alpha)
    elif color_initial == "y":
        return ColorRGBA(r=1.0, g=1.0, a=alpha)
    elif color_initial == "c":
        return ColorRGBA(g=1.0, b=1.0, a=alpha)
    elif color_initial == "w":
        return ColorRGBA(r=1.0, g=1.0, b=1.0, a=alpha)
    else:
        return ColorRGBA(a=alpha)


def get_random_color_rgba(alpha: float = 1.0) -> ColorRGBA:
    return ColorRGBA(
        r=np.random.random(),
        g=np.random.random(),
        b=np.random.random(),
        a=alpha,
    )
