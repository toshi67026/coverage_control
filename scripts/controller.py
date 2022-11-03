#!/usr/bin/env python

import traceback

import rospy
from geometry_msgs.msg import Pose, Twist, Vector3


class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller")

        self.kp = float(rospy.get_param("/kp", default=1))
        timer_period = float(rospy.get_param("/timer_period", default=0.1))

        self.ref_pose = Pose()
        self.curr_pose = Pose()
        self.cmd_vel_in_world = Twist()

        # pub
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # sub
        rospy.Subscriber("ref_pose", Pose, self.ref_pose_callback, queue_size=1)
        rospy.Subscriber("curr_pose", Pose, self.curr_pose_callback, queue_size=1)

        # timer
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

    def curr_pose_callback(self, msg: Pose) -> None:
        self.curr_pose = msg

    def ref_pose_callback(self, msg: Pose) -> None:
        self.ref_pose = msg

    def timer_callback(self, timer: rospy.Timer) -> None:
        # 偏差
        x_err = self.ref_pose.position.x - self.curr_pose.position.x
        y_err = self.ref_pose.position.y - self.curr_pose.position.y
        z_err = self.ref_pose.position.z - self.curr_pose.position.z

        # p-control
        self.cmd_vel_in_world.linear = Vector3(
            x=self.kp * x_err,
            y=self.kp * y_err,
            z=self.kp * z_err,
        )
        self.cmd_vel_pub.publish(self.cmd_vel_in_world)


def main() -> None:
    try:
        Controller()
        rospy.spin()
    except:
        rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    main()
