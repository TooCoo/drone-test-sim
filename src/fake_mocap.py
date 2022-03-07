#!/usr/bin/env python3
import roslib
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf


def pose_forwarding(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                     [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w],
                     rospy.Time.now(),
                     "base_link",
                     "map")


# sub to: /mavros/local_position/pose
if __name__ == '__main__':
    rospy.init_node('fake_mocap_node')
    rospy.Subscriber('/mavros/local_position/pose',
                     PoseStamped,
                     pose_forwarding)

    rospy.spin()

