#!/usr/bin/env python3
# WIFI CONFIG
# SSID: OptiTrack
# PSW: 60A84A244BECD

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from copy import deepcopy
import math
import sys
import tf

# Import your existing NatNet client (same as in ROS2 code)
from nat_net_client import NatNetClient

# IMPORTANT: can be overridden via ROS param ~tracked_robot_id
ROBOT_ID_DEFAULT = 20

class Optitrack(object):
    def __init__(self, debug=True, ns="limo0"):
        node_name = 'optitrack_node'
        rospy.loginfo('%s starting (ROS1 port) - version 2.2', node_name)


        self.ns = ns
        self.pub_odom  = rospy.Publisher('/'+self.ns+'/odom',  Odometry, queue_size=10)
        self.odom_msg = Odometry()

        self.publisher_frequency = 100.0  # Hz
        self.pub_timer = rospy.Timer(rospy.Duration(1.0 / self.publisher_frequency), self.pub_timer_callback)

        self.actual_pose      = PoseStamped()
        self.actual_pose_old  = PoseStamped()
        self.feasible_pose    = PoseStamped()
        self.twist_msg        = Twist()
        self.vel              = np.zeros(3)
        self.omega            = np.zeros(3)
        self.debug            = debug
        self.calls            = 0
        self.total_calls      = 0

        self.broadcaster = tf.TransformBroadcaster()

        if self.debug:
            self.debug_timer = rospy.Timer(rospy.Duration(1.0), self.debug_timer_callback)

        # Params
        self.marker_robot_id =  ROBOT_ID_DEFAULT

        # Start NatNet streaming client
        streamingClient = NatNetClient(ver=(3, 1, 0, 0), quiet=True)
        streamingClient.rigidBodyListener = self.receiveRigidBodyFrame
        streamingClient.run()
        print("optitrack node init successfull")

    # --- Math helpers (unchanged logic) ---
    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def computeOmega(self, q, q_old, dt):
        q1 = np.array([q.w, q.x, q.y, q.z])
        q2 = np.array([q_old.w, -q_old.x, -q_old.y, -q_old.z])
        work = (2.0 / dt) * self.quatMultiply(q1, q2)
        return work[1:]  # imaginary part

    def quatMultiply(self, q, r):
        s1 = q[0]
        s2 = r[0]
        v1 = q[1:]
        v2 = r[1:]
        vec = s1 * v2 + s2 * v1 + np.cross(v1, v2)
        scalar = s1 * s2 - np.dot(v1, v2)
        return np.concatenate(([scalar], vec))

    def getTimeInterval(self, t1, t2):
        # rospy.Time supports subtraction -> rospy.Duration
        try:
            return (t1 - t2).to_sec()
        except Exception:
            # In case one is zero-initialized
            return 0.0

    # --- Timers ---
    def debug_timer_callback(self, event):
        rospy.loginfo("Hz: %d", self.calls)
        self.calls = 0

    def pub_timer_callback(self, event):
        # Publish only after at least one OptiTrack frame was received
        if self.total_calls >= 1:
            # Copy current pose to feasible pose
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.header.frame_id = '/' + self.ns + '/odom'
            self.odom_msg.child_frame_id = '/' + self.ns + '/base_link'
            self.odom_msg.pose.pose.position.x = self.actual_pose.pose.position.x
            self.odom_msg.pose.pose.position.y = self.actual_pose.pose.position.y
            self.odom_msg.pose.pose.position.z = self.actual_pose.pose.position.z
            self.odom_msg.pose.pose.orientation.w = self.actual_pose.pose.orientation.w
            self.odom_msg.pose.pose.orientation.x = self.actual_pose.pose.orientation.x
            self.odom_msg.pose.pose.orientation.y = self.actual_pose.pose.orientation.y
            self.odom_msg.pose.pose.orientation.z = self.actual_pose.pose.orientation.z
            self.odom_msg.twist.twist.linear.x  = float(self.vel[0])
            self.odom_msg.twist.twist.linear.y  = float(self.vel[1])
            self.odom_msg.twist.twist.linear.z  = float(self.vel[2])
            self.odom_msg.twist.twist.angular.x = float(self.omega[0])
            self.odom_msg.twist.twist.angular.y = float(self.omega[1])
            self.odom_msg.twist.twist.angular.z = float(self.omega[2])
            self.pub_odom.publish(self.odom_msg)

            #send TF from odom to baseframe
            self.broadcaster.sendTransform(np.array([ self.actual_pose.pose.position.x,  self.actual_pose.pose.position.y,  self.actual_pose.pose.position.z]),
                                           np.array([
                                               self.actual_pose.pose.orientation.x,
                                               self.actual_pose.pose.orientation.y,
                                               self.actual_pose.pose.orientation.z,
                                               self.actual_pose.pose.orientation.w
                                           ]),
                                           rospy.Time.now(), '/'+self.ns+'/base_link', '/'+self.ns+'/odom')

    # --- NatNet callback (runs in client thread) ---
    def receiveRigidBodyFrame(self, rid, position, rotation):
        if rid != self.marker_robot_id:
            rospy.logdebug('Message with different tracker ID: %s', str(rid))
            return

        # Header
        self.actual_pose.header.frame_id = "tag"
        self.actual_pose.header.stamp = rospy.Time.now()
        self.actual_pose.pose.position.x = position[0]
        self.actual_pose.pose.position.y = position[1]
        self.actual_pose.pose.position.z = position[2]

        # Orientation: NatNet gives (qx, qy, qz, qw)
        self.actual_pose.pose.orientation.w = rotation[3]     # q_w
        self.actual_pose.pose.orientation.x = rotation[0]     # q_x
        self.actual_pose.pose.orientation.y = rotation[1]    # q_y
        self.actual_pose.pose.orientation.z = rotation[2]     # q_z

        # Flip quaternion if long path
        old_quat = np.array([self.feasible_pose.pose.orientation.w,
                             self.feasible_pose.pose.orientation.x,
                             self.feasible_pose.pose.orientation.y,
                             self.feasible_pose.pose.orientation.z])
        new_quat = np.array([self.actual_pose.pose.orientation.w,
                             self.actual_pose.pose.orientation.x,
                             self.actual_pose.pose.orientation.y,
                             self.actual_pose.pose.orientation.z])
        if np.dot(old_quat, new_quat) < 0.0:
            self.actual_pose.pose.orientation.w *= -1.0
            self.actual_pose.pose.orientation.x *= -1.0
            self.actual_pose.pose.orientation.y *= -1.0
            self.actual_pose.pose.orientation.z *= -1.0

        # Compute velocities (skip first point / zero dt)
        dt = self.getTimeInterval(self.actual_pose.header.stamp, self.actual_pose_old.header.stamp)
        if dt > 0.0:
            old_pos = np.array([self.actual_pose_old.pose.position.x,
                                self.actual_pose_old.pose.position.y,
                                self.actual_pose_old.pose.position.z])
            act_pos = np.array([self.actual_pose.pose.position.x,
                                self.actual_pose.pose.position.y,
                                self.actual_pose.pose.position.z])

            _, _, y_new = self.euler_from_quaternion(self.actual_pose.pose.orientation)
            _, _, y_old = self.euler_from_quaternion(self.actual_pose_old.pose.orientation)

            # Keep same behavior as your ROS2 code (use publisher_frequency)
            self.vel = (act_pos - old_pos) * self.publisher_frequency
            self.omega[:] = 0.0
            self.omega[2] = (y_new - y_old) * self.publisher_frequency

            self.actual_pose_old = deepcopy(self.actual_pose)
        else:
            # Initialize the "old" pose on very first frame
            self.actual_pose_old = deepcopy(self.actual_pose)

        self.calls += 1
        self.total_calls += 1

if __name__ == '__main__':
    rospy.init_node('optitrack_node', anonymous=False)

    limo_ns = rospy.get_param("~ns", "limo0")
    args = rospy.myargv(argv=sys.argv)
    debug = '--debug' in args

    node = Optitrack(debug=debug, ns=limo_ns)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    rospy.loginfo("Shutting down optitrack_node")