#!/usr/bin/env python3
# ==========================================================
# FASTLIO / LIVOX ODOMETRY -> MAVROS ONLY (FIXED)
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
camera_orientation_default = 0  # 0=Forward, 1=Downward, 2=45deg tilt
scale_factor = 1.0
debug_enable = 1

# ==========================================================
# FIXED: PROPER ENU TO NED TRANSFORMATION
# ==========================================================

# ENU (ROS standard) to NED (ArduPilot/Mavros standard)
# X_ENU -> Y_NED, Y_ENU -> X_NED, Z_ENU -> -Z_NED
R_ENU_to_NED = np.array([
    [0,  1,  0],
    [1,  0,  0],
    [0,  0, -1]
])

# Full 4x4 transformation
T_ENU_to_NED = np.eye(4)
T_ENU_to_NED[:3, :3] = R_ENU_to_NED

# ==========================================================
# CAMERA ORIENTATION OFFSETS (relative to forward)
# ==========================================================

def get_camera_rotation_matrix(orientation):
    """Get rotation from camera frame to body frame based on mount orientation"""
    if orientation == 0:  # Forward facing
        # Camera X=forward, Y=right, Z=down (standard camera frame)
        # to Body X=forward, Y=right, Z=down
        return np.eye(3)
        
    elif orientation == 1:  # Downward facing
        # Camera looks down: rotate +90 deg around X (pitch down)
        return np.array([
            [1,  0,  0],
            [0,  0,  1],
            [0, -1,  0]
        ])
        
    elif orientation == 2:  # 45 degree tilt down
        angle = m.pi / 4
        return np.array([
            [1,      0,       0],
            [0, m.cos(angle), m.sin(angle)],
            [0, -m.sin(angle), m.cos(angle)]
        ])
        
    else:
        return np.eye(3)

# Build full transformation matrices
R_cam_to_body = get_camera_rotation_matrix(camera_orientation_default)
T_cam_to_body = np.eye(4)
T_cam_to_body[:3, :3] = R_cam_to_body

# Combined: ENU -> NED -> Body
T_ENU_to_Body = T_cam_to_body.dot(T_ENU_to_NED)

# ==========================================================
# CALLBACK
# ==========================================================

def odom_callback(msg):
    global pub_pose, pub_speed

    # --------------------------------------------------
    # Extract input pose (ENU frame from FASTLIO)
    # --------------------------------------------------
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    # Build 4x4 transform from odometry
    T_enu = tf.quaternion_matrix([qx, qy, qz, qw])
    T_enu[0, 3] = msg.pose.pose.position.x * scale_factor
    T_enu[1, 3] = msg.pose.pose.position.y * scale_factor
    T_enu[2, 3] = msg.pose.pose.position.z * scale_factor

    # --------------------------------------------------
    # FIXED: Proper ENU -> NED transformation
    # --------------------------------------------------
    # T_ned = T_ENU_to_NED * T_enu * T_ENU_to_NED^T for rotation
    # For full transform: apply rotation, then translation transform
    T_ned = T_ENU_to_NED.dot(T_enu).dot(np.linalg.inv(T_ENU_to_NED))
    
    # Alternative simpler method for position+orientation:
    # Just rotate everything through ENU->NED
    pos_ned = R_ENU_to_NED.dot(np.array([
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    ])) * scale_factor
    
    # For orientation: R_ned = R_ENU_to_NED * R_enu * R_NED_to_ENU
    R_enu = T_enu[:3, :3]
    R_ned = R_ENU_to_NED.dot(R_enu).dot(R_ENU_to_NED.T)
    
    # Rebuild full matrix with correct position
    T_ned = np.eye(4)
    T_ned[:3, :3] = R_ned
    T_ned[:3, 3] = pos_ned

    # --------------------------------------------------
    # Apply camera-to-body rotation if needed
    # --------------------------------------------------
    T_final = T_cam_to_body.dot(T_ned)

    # --------------------------------------------------
    # Extract Position
    # --------------------------------------------------
    pos = tf.translation_from_matrix(T_final)

    # --------------------------------------------------
    # Extract Quaternion
    # --------------------------------------------------
    # q_out = tf.quaternion_from_matrix(T_final)  # ORIENTATION COMMENTED OUT

    # --------------------------------------------------
    # Publish Pose
    # --------------------------------------------------
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"

    pose_msg.pose.position.x = pos[0]
    pose_msg.pose.position.y = pos[1]
    pose_msg.pose.position.z = pos[2]

    # pose_msg.pose.orientation.x = q_out[0]  # ORIENTATION COMMENTED OUT
    # pose_msg.pose.orientation.y = q_out[1]  # ORIENTATION COMMENTED OUT
    # pose_msg.pose.orientation.z = q_out[2]  # ORIENTATION COMMENTED OUT
    # pose_msg.pose.orientation.w = q_out[3]  # ORIENTATION COMMENTED OUT

    pub_pose.publish(pose_msg)

    # --------------------------------------------------
    # FIXED: Velocity transformation
    # --------------------------------------------------
    # Transform linear velocity from ENU to NED
    v_enu = np.array([
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z
    ])
    v_ned = R_ENU_to_NED.dot(v_enu)
    
    # Then apply camera rotation if needed
    v_body = R_cam_to_body.dot(v_ned)

    twist_msg = TwistStamped()
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "map"

    twist_msg.twist.linear.x = v_body[0]
    twist_msg.twist.linear.y = v_body[1]
    twist_msg.twist.linear.z = v_body[2]

    # Angular velocity: same rotation applies
    # w_enu = np.array([  # ANGULAR VEL COMMENTED OUT
        # msg.twist.twist.angular.x,  # ANGULAR VEL COMMENTED OUT
        # msg.twist.twist.angular.y,  # ANGULAR VEL COMMENTED OUT
        # msg.twist.twist.angular.z  # ANGULAR VEL COMMENTED OUT
    # ])  # ANGULAR VEL COMMENTED OUT
    # w_ned = R_ENU_to_NED.dot(w_enu)  # ANGULAR VEL COMMENTED OUT
    # w_body = R_cam_to_body.dot(w_ned)  # ANGULAR VEL COMMENTED OUT

    # twist_msg.twist.angular.x = w_body[0]  # ANGULAR VEL COMMENTED OUT
    # twist_msg.twist.angular.y = w_body[1]  # ANGULAR VEL COMMENTED OUT
    # twist_msg.twist.angular.z = w_body[2]  # ANGULAR VEL COMMENTED OUT

    pub_speed.publish(twist_msg)

    # --------------------------------------------------
    # Debug - FIXED: Use proper Euler order for NED
    # --------------------------------------------------
    # if debug_enable:
        # # For NED frame, use 'rzyx' (ZYX) order: Yaw-Pitch-Roll
        # # or 'sxyz' depending on your convention
        # rpy = np.array(
            # tf.euler_from_matrix(T_final, 'sxyz')  # NED convention
        # ) * 180 / np.pi

        # rospy.loginfo_throttle(1.0,
            # "POS: x=%.2f y=%.2f z=%.2f | RPY[deg]: Roll=%.2f Pitch=%.2f Yaw=%.2f",
            # pos[0], pos[1], pos[2],
            # rpy[0], rpy[1], rpy[2]
        # )  # DEBUG LOG COMMENTED OUT

# ==========================================================
# MAIN
# ==========================================================

if __name__ == "__main__":

    rospy.init_node("fastlio_mavros_bridge_fixed")

    odom_topic = rospy.get_param("~odom_topic", odom_topic_default)
    cam_orient = rospy.get_param("~camera_orientation", camera_orientation_default)

    # Update camera transform if parameter changed
    if cam_orient != camera_orientation_default:
        R_cam_to_body = get_camera_rotation_matrix(cam_orient)
        T_cam_to_body[:3, :3] = R_cam_to_body
        T_ENU_to_Body = T_cam_to_body.dot(T_ENU_to_NED)

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

    rospy.loginfo("FASTLIO -> MAVROS bridge (FIXED) started")
    rospy.loginfo("Subscribed: %s", odom_topic)
    rospy.loginfo("Camera orientation: %d", cam_orient)

    rospy.spin()
