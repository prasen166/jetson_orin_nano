#!/usr/bin/env python3
# ==========================================================
# FASTLIO / LIVOX ODOMETRY -> MAVROS ONLY
# Publishes:
#   /mavros/vision_pose/pose
#   /mavros/vision_speed/speed_twist
# Input:
#   /Odometry   (nav_msgs/Odometry)
# ==========================================================

import rospy
import numpy as np
import math as m
import tf.transformations as tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped

# ==========================================================
# CONFIG
# ==========================================================

odom_topic_default = "/Odometry"

# Camera orientation:
# 0 = Forward
# 1 = Downward
# 2 = 45 deg tilt
camera_orientation_default = 0

scale_factor = 1.0
debug_enable = 1

# ==========================================================
# GLOBALS
# ==========================================================

pub_pose = None
pub_speed = None

# ==========================================================
# ARGUMENTS
# ==========================================================

camera_orientation = camera_orientation_default

# ==========================================================
# CAMERA ORIENTATION MATRICES
# ==========================================================

if camera_orientation == 0:
    H_aeroRef_sensorRef = np.array([
        [0, 0,-1,0],
        [1, 0, 0,0],
        [0,-1, 0,0],
        [0, 0, 0,1]
    ])
    H_sensorBody_aeroBody = np.linalg.inv(H_aeroRef_sensorRef)

elif camera_orientation == 1:
    H_aeroRef_sensorRef = np.array([
        [0,0,-1,0],
        [1,0,0,0],
        [0,-1,0,0],
        [0,0,0,1]
    ])
    H_sensorBody_aeroBody = np.array([
        [0,1,0,0],
        [1,0,0,0],
        [0,0,-1,0],
        [0,0,0,1]
    ])

elif camera_orientation == 2:
    H_aeroRef_sensorRef = np.array([
        [0,0,-1,0],
        [1,0,0,0],
        [0,-1,0,0],
        [0,0,0,1]
    ])
    H_sensorBody_aeroBody = tf.euler_matrix(m.pi/4,0,0).dot(
        np.linalg.inv(H_aeroRef_sensorRef)
    )

else:
    H_aeroRef_sensorRef = np.array([
        [0,0,-1,0],
        [1,0,0,0],
        [0,-1,0,0],
        [0,0,0,1]
    ])
    H_sensorBody_aeroBody = np.linalg.inv(H_aeroRef_sensorRef)

# ==========================================================
# CALLBACK
# ==========================================================

def odom_callback(msg):
    global pub_pose, pub_speed

    # --------------------------------------------------
    # Input Quaternion (ROS = x,y,z,w)
    # --------------------------------------------------
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    H_sensorRef_sensorBody = tf.quaternion_matrix(
        [qx, qy, qz, qw]
    )

    # Position
    H_sensorRef_sensorBody[0][3] = msg.pose.pose.position.x * scale_factor
    H_sensorRef_sensorBody[1][3] = msg.pose.pose.position.y * scale_factor
    H_sensorRef_sensorBody[2][3] = msg.pose.pose.position.z * scale_factor

    # --------------------------------------------------
    # Convert ENU -> NED
    # --------------------------------------------------
    H_aeroRef_aeroBody = H_aeroRef_sensorRef.dot(
        H_sensorRef_sensorBody.dot(
            H_sensorBody_aeroBody
        )
    )

    # --------------------------------------------------
    # Extract Position
    # --------------------------------------------------
    pos = tf.translation_from_matrix(H_aeroRef_aeroBody)

    # --------------------------------------------------
    # Extract Quaternion
    # --------------------------------------------------
    q_out = tf.quaternion_from_matrix(H_aeroRef_aeroBody)

    # --------------------------------------------------
    # Publish Pose
    # --------------------------------------------------
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"

    pose_msg.pose.position.x = pos[0]
    pose_msg.pose.position.y = pos[1]
    pose_msg.pose.position.z = pos[2]

    pose_msg.pose.orientation.x = q_out[0]
    pose_msg.pose.orientation.y = q_out[1]
    pose_msg.pose.orientation.z = q_out[2]
    pose_msg.pose.orientation.w = q_out[3]

    pub_pose.publish(pose_msg)

    # --------------------------------------------------
    # Velocity ENU -> NED
    # --------------------------------------------------
    vx = msg.twist.twist.linear.y
    vy = msg.twist.twist.linear.x
    vz = -msg.twist.twist.linear.z

    twist_msg = TwistStamped()
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "map"

    twist_msg.twist.linear.x = vx
    twist_msg.twist.linear.y = vy
    twist_msg.twist.linear.z = vz

    twist_msg.twist.angular.x = msg.twist.twist.angular.x
    twist_msg.twist.angular.y = msg.twist.twist.angular.y
    twist_msg.twist.angular.z = msg.twist.twist.angular.z

    pub_speed.publish(twist_msg)

    # --------------------------------------------------
    # Debug
    # --------------------------------------------------
    if debug_enable:
        rpy = np.array(
            tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz')
        ) * 180 / np.pi

        rospy.loginfo_throttle(1.0,
            "RPY[deg]: Roll=%.2f Pitch=%.2f Yaw=%.2f",
            rpy[0], rpy[1], rpy[2]
        )

# ==========================================================
# MAIN
# ==========================================================

if __name__ == "__main__":

    rospy.init_node("fastlio_mavros_bridge")

    odom_topic = rospy.get_param("~odom_topic", odom_topic_default)

    pub_pose = rospy.Publisher(
        "/mavros/vision_pose/pose",
        PoseStamped,
        queue_size=20
    )

    pub_speed = rospy.Publisher(
        "/mavros/vision_speed/speed_twist",
        TwistStamped,
        queue_size=20
    )

    rospy.Subscriber(
        odom_topic,
        Odometry,
        odom_callback,
        queue_size=50
    )

    rospy.loginfo("FASTLIO -> MAVROS bridge started")
    rospy.loginfo("Subscribed: %s", odom_topic)

    rospy.spin()
