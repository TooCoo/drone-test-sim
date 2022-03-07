#!/usr/bin/env python3
# license removed for brevity

#rostopic echo /mavros/vision_pose/pose

import numpy as np
from copy import copy
import rospy
from mavros_msgs.msg import State,Altitude, HomePosition, ExtendedState, PositionTarget
from mavros_msgs.srv import CommandBool,SetMode,CommandTOL, ParamGet, StreamRateRequest,StreamRate
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

# from trajectory_utility import *

# /vive/LHR_6C62CAC0_pose /mavros/vision_pose/pose


class PosePublisher:
    def __init__(self):
        rospy.init_node('vive_to_mavros', anonymous=True)
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

        self.state_sub = rospy.Subscriber('/mavros/state',State,self.state_cb)

        self.state = State()
        self.mode = ''

        self.waypoint_frame = 'map'

        # Frame of target position
        self.start_position.header.frame_id = self.waypoint_frame

        # Starting pose and orientation, orientation is a quaternion
        self.start_position.pose.position.x = 0.3
        self.start_position.pose.position.y = 0.3
        self.start_position.pose.position.z = 0.8

        self.start_position.pose.orientation.x = 0
        self.start_position.pose.orientation.y = 0
        self.start_position.pose.orientation.z = 0
        self.start_position.pose.orientation.w = 1

        self.waypoints = [self.start_position]

        # Waypoints are copied, make individual waypoints

        wp_a = PoseStamped()
        wp_a.header.frame_id = self.waypoint_frame

        wp_a.pose.position.x = 0.3
        wp_a.pose.position.y = 0.3 - 0.5
        wp_a.pose.position.z = 0.8

        wp_a.pose.orientation.x = 0
        wp_a.pose.orientation.y = 0
        wp_a.pose.orientation.z = 0
        wp_a.pose.orientation.w = 1

        self.waypoints.append(wp_a)

        wp_b = PoseStamped()
        wp_b.header.frame_id = self.waypoint_frame

        wp_b.pose.position.x = 0.3 + 0.5
        wp_b.pose.position.y = 0.3 - 0.5
        wp_b.pose.position.z = 0.8

        wp_b.pose.orientation.x = 0
        wp_b.pose.orientation.y = 0
        wp_b.pose.orientation.z = 0
        wp_b.pose.orientation.w = 1
        self.waypoints.append(wp_b)

        wp_c = PoseStamped()
        wp_c.header.frame_id = self.waypoint_frame

        wp_c.pose.position.x = 0.3 + 0.5
        wp_c.pose.position.y = 0.3 + 0.5
        wp_c.pose.position.z = 0.8

        wp_c.pose.orientation.x = 0
        wp_c.pose.orientation.y = 0
        wp_c.pose.orientation.z = 0
        wp_c.pose.orientation.w = 1
        self.waypoints.append(wp_c)

        wp_d = PoseStamped()
        wp_d.header.frame_id = self.waypoint_frame

        wp_d.pose.position.x = 0.3
        wp_d.pose.position.y = 0.3 + 0.5
        wp_d.pose.position.z = 0.8

        wp_d.pose.orientation.x = 0
        wp_d.pose.orientation.y = 0
        wp_d.pose.orientation.z = 0
        wp_d.pose.orientation.w = 1
        self.waypoints.append(wp_d)

        # Finish where we started
        self.waypoints.append(self.start_position)

        self.current_wp = 0
        self.arrive_tol = 0.1

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
        target_position = self.waypoints[self.current_wp].pose.position
        distance_vector = [target_position.x - current_position.x, target_position.y - current_position.y, target_position.z - current_position.z]
        distance = np.linalg.norm(distance_vector)

        if distance < self.arrive_tol:
            self.current_wp += 1

        #print(f"{msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")

    def run(self):
        # rospy.init_node('vive_to_mavros', anonymous=True)
        # self.pose_publisher = rospy.Publisher(self.repeat_name, PoseStamped, queue_size=10)
        rospy.Subscriber(self.current_pose_name, PoseStamped, self.callback)
        rospy.loginfo('Pose publisher has started.')

        self.start_offboard()

        count = 0

        while not rospy.is_shutdown():
            count = count + 1

            if self.current_wp == len(self.waypoints):
                print("This is the final waypoint.")
            else:
                msg = self.waypoints[self.current_wp]
                msg.header.seq = count
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'map'

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
