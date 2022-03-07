#!/usr/bin/env python3
# license removed for brevity

#rostopic echo /mavros/vision_pose/pose

import numpy as np

import rospy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# from trajectory_utility import *

# /vive/LHR_6C62CAC0_pose /mavros/vision_pose/pose


class PosePublisher:
    def __init__(self):
        rospy.init_node('vive_to_mavros', anonymous=True)
        self.rate = rospy.Rate(30)  # 10hz
        self.topic_name = '/mavros/setpoint_position/local'
        self.pose_publisher = rospy.Publisher(self.topic_name, PoseStamped, queue_size=10)

    def run(self):
        # rospy.init_node('vive_to_mavros', anonymous=True)
        # self.pose_publisher = rospy.Publisher(self.repeat_name, PoseStamped, queue_size=10)
        rospy.loginfo('Pose publisher has started.')
        msg = PoseStamped()
        msg.header.frame_id = 'map'

        msg.pose.position.x = 0.3
        msg.pose.position.y = 0.3
        msg.pose.position.z = 0.8
        msg.pose.orientation.w = 1

        count = 0

        while not rospy.is_shutdown():
            count = count + 1
            msg.header.seq = count
            msg.header.stamp = rospy.Time.now()
            self.pose_publisher.publish(msg)
            self.rate.sleep()
        

if __name__ == '__main__':
    try:
        PosePublisher().run()
    except rospy.ROSInterruptException:
        pass
