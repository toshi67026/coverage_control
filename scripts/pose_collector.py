#!/usr/bin/env python

import traceback
from dataclasses import dataclass
from typing import List

import rospy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header


@dataclass
class Data:
    curr_pose: Pose = Pose()
    is_ready: bool = False


class PoseCollector:
    def __init__(self) -> None:
        rospy.init_node("pose_collector")

        self.world_frame = str(rospy.get_param("/world_frame", default="world"))
        agent_num = int(rospy.get_param("/agent_num", default=-1))
        agent_prefix = str(rospy.get_param("/agent_prefix", default="agent_"))

        self.data_list: List[Data] = [Data()] * agent_num
        self.is_ready = False

        timer_period = float(rospy.get_param("/timer_period", default=0.1))

        # pub
        self.curr_pose_array_pub = rospy.Publisher("/curr_pose_array", PoseArray, queue_size=1)

        # sub
        for agent_id in range(agent_num):
            agent_name = agent_prefix + str(agent_id)
            topic_name = agent_name + "/curr_pose"
            rospy.Subscriber(topic_name, Pose, self.curr_pose_callback, callback_args=agent_id)

        # timer
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

    def curr_pose_callback(self, msg: Pose, agent_id: int) -> None:
        self.data_list[agent_id] = Data(curr_pose=msg, is_ready=True)

    def timer_callback(self, timer: rospy.Timer) -> None:
        # 全agentのcurr_poseが揃い，is_ready==Trueとなるまでpublishしない
        if self.is_ready:
            curr_pose_array = [data.curr_pose for data in self.data_list]
            self.curr_pose_array_pub.publish(
                PoseArray(header=Header(stamp=rospy.Time.now(), frame_id=self.world_frame), poses=curr_pose_array)
            )
        else:
            if len([data.is_ready for data in self.data_list if not data.is_ready]) == 0:
                rospy.logwarn("pose_collector is ready")
                self.is_ready = True


def main() -> None:
    try:
        PoseCollector()
        rospy.spin()
    except:
        rospy.logerr(traceback.format_exc())


if __name__ == "__main__":
    main()
