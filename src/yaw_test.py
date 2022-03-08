#!/usr/bin/env python3
# license removed for brevity

# rostopic echo /mavros/vision_pose/pose

import numpy as np
from copy import copy
import rospy
from mavros_msgs.msg import State, Altitude, HomePosition, ExtendedState, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, ParamGet, StreamRateRequest, StreamRate
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion

from tf.transformations import quaternion_from_euler


# from trajectory_utility import *

# /vive/LHR_6C62CAC0_pose /mavros/vision_pose/pose


class PosePublisher:
    def __init__(self):
        rospy.init_node('pose_demo_node', anonymous=True)
        self.rate = rospy.Rate(30)  # 10hz
        self.current_pose_name = '/mavros/local_position/pose'
        self.topic_name = '/mavros/setpoint_position/local'
        self.pose_publisher = rospy.Publisher(self.topic_name, PoseStamped, queue_size=10)
        self.start_position = PoseStamped()

        self.extended_state = ExtendedState()
        self.mode = ''
        self.current_yaw = 0

        # Setup services to set commands to drone
        self.service_timeout = 30

        rospy.wait_for_service('/mavros/cmd/arming', self.service_timeout)
        rospy.wait_for_service('/mavros/set_mode', self.service_timeout)
        rospy.wait_for_service('/mavros/cmd/takeoff', self.service_timeout)
        rospy.wait_for_service('/mavros/cmd/land', self.service_timeout)

        rospy.loginfo('Services are connected and ready')

        self.set_arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.set_land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)

        self.state = State()
        self.mode = ''

        self.waypoint_frame = 'map'

        # Frame of target position
        self.target_position = PoseStamped()
        self.target_position.header.frame_id = self.waypoint_frame

        # Starting pose and orientation, orientation is a quaternion
        self.target_position.pose.position.x = 0.3
        self.target_position.pose.position.y = 0.3
        self.target_position.pose.position.z = 0.8

        self.target_position.pose.orientation.x = 0
        self.target_position.pose.orientation.y = 0
        self.target_position.pose.orientation.z = 0
        self.target_position.pose.orientation.w = 1

        self.distance_inc = 0.5
        self.travel_range = 3.0
        self.travel_direction = 1

        self.current_yaw = 0
        self.d_yaw = 0.01

        self.arrive_tol = 0.15

    def state_cb(self, data):
        self.state = data
        self.mode = data.mode

    def set_mode(self, mode):
        if self.state.mode != mode:
            try:
                mode_change_response = self.set_mode_srv(base_mode=0, custom_mode=mode)
                last_request_time = rospy.get_rostime()
                if not mode_change_response.mode_sent:
                    rospy.logerr('---Mode change failed---')
            except rospy.ServiceException as exception:
                rospy.logerr('Failed to change mode')

    def arm(self):
        last_request_time = rospy.get_rostime()
        if not self.state.armed:
            arm_response = self.set_arm_srv(True)
            if arm_response:
                rospy.loginfo('---Vehicle armed --')
            else:
                rospy.loginfo('---Arming failed ---')
            last_request_time = rospy.get_rostime()
        else:
            # vehicle is already armed
            pass

    def disarm(self):
        if self.set_arm_srv(False):
            rospy.loginfo('--Vehicle disarmed---')
        else:
            rospy.loginfo('---Disarming failed')

    def start_offboard(self):
        # wait to get heartbeat from fcu
        while not self.state.connected:
            self.rate.sleep()

        rospy.loginfo('--Got heartbeat from FCU----')

    def state_cb(self, data):
        self.state = data
        self.mode = data.mode

    def callback(self, msg):
        distance_to_next_point = 0
        current_position = msg.pose.position
        target_position = self.target_position.pose.position
        distance_vector = [target_position.x - current_position.x, target_position.y - current_position.y,
                           target_position.z - current_position.z]

        distance = np.linalg.norm(distance_vector)

        if distance < self.arrive_tol:
            # If we are at the last waypoing fine

            if current_position.y > self.travel_range:
                self.travel_direction = -1
            elif current_position.y < - self.travel_range:
                self.travel_direction = 1

            # self.target_position.pose.position.y = self.target_position.pose.position.y + self.distance_inc * self.travel_direction
            # print(f"{msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")

    def run(self):
        # rospy.init_node('vive_to_mavros', anonymous=True)
        # self.pose_publisher = rospy.Publisher(self.repeat_name, PoseStamped, queue_size=10)
        rospy.Subscriber(self.current_pose_name, PoseStamped, self.callback)
        rospy.loginfo('Pose publisher has started.')

        self.start_offboard()

        count = 0

        while not rospy.is_shutdown():
            count = count + 1

            msg = self.target_position
            msg.header.seq = count
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'

            self.current_yaw += self.d_yaw
            new_orientation = quaternion_from_euler(0, 0, self.current_yaw)

            msg.pose.orientation.x = new_orientation[0]
            msg.pose.orientation.y = new_orientation[1]
            msg.pose.orientation.z = new_orientation[2]
            msg.pose.orientation.w = new_orientation[3]

            self.pose_publisher.publish(msg)

            if count == 50:
                self.arm()
                self.set_mode("OFFBOARD")

            self.rate.sleep()


if __name__ == '__main__':
    try:
        PosePublisher().run()
    except rospy.ROSInterruptException:
        pass
