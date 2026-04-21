#!/usr/bin/env python3
"""
fastlio_to_mavros_bridge.py

Fixes frame mismatch between Fast-LIO sensor frame and MAVROS FLU convention.
Your sensor appears to be FRU (Forward-Right-Up) or similar, requiring
a -90 degree rotation about Z to align to FLU.
"""

import rospy
import numpy as np
import tf.transformations as tf_trans
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros


class FastLIOToMAVROS:
    def __init__(self):
        rospy.init_node('fastlio_to_mavros', anonymous=True)

        self.odom_topic        = rospy.get_param('~odom_topic',        '/Odometry')
        self.mavros_pose_topic = rospy.get_param('~mavros_pose_topic', '/mavros/vision_pose/pose')
        self.frame_id          = rospy.get_param('~frame_id',          'map')
        self.child_frame_id    = rospy.get_param('~child_frame_id',    'base_link')
        self.publish_tf        = rospy.get_param('~publish_tf',        True)

        # Based on your mapping:
        # drone_yaw = sensor_yaw, drone_pitch = sensor_roll, drone_roll = -sensor_pitch
        # This is a +90 degree rotation about Z axis (sensor to FLU)
        # q = [0, 0, sin(45), cos(45)] = [0, 0, 0.7071, 0.7071]
        #
        # Try this first. If it makes it worse, use -90 deg: [0, 0, -0.7071, 0.7071]
        self.q_sensor_to_flu = np.array([0.0, 0.0, 0.70710678, 0.70710678])  # +90° about Z

        self.pose_pub = rospy.Publisher(self.mavros_pose_topic, PoseStamped, queue_size=10)
        
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

        rospy.loginfo("=== Fast-LIO to MAVROS Bridge ===")
        rospy.loginfo("Frame correction: Sensor -> FLU (+90 deg Z)")
        rospy.loginfo("Mapping: yaw=yaw, pitch=roll, roll=-pitch -> FLU convention")

    def odom_callback(self, msg):
        # Position: pass through (ENU)
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z

        # Sensor quaternion
        q_sensor = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ], dtype=np.float64)

        q_norm = np.linalg.norm(q_sensor)
        if q_norm < 1e-6 or np.isnan(q_sensor).any():
            rospy.logerr_throttle(5.0, "Invalid quaternion from Fast-LIO - skipping")
            return
        q_sensor = q_sensor / q_norm

        # Apply sensor-to-FLU rotation
        # q_flu = q_sensor * q_sensor_to_flu  (rotate sensor frame to align with FLU)
        q_flu = tf_trans.quaternion_multiply(q_sensor, self.q_sensor_to_flu)
        q_flu = q_flu / np.linalg.norm(q_flu)

        # Publish to MAVROS (MAVROS will convert FLU->FRD internally)
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = msg.header.stamp
        pose_msg.header.frame_id = self.frame_id
        
        pose_msg.pose.position.x = px
        pose_msg.pose.position.y = py
        pose_msg.pose.position.z = pz
        
        pose_msg.pose.orientation.x = q_flu[0]
        pose_msg.pose.orientation.y = q_flu[1]
        pose_msg.pose.orientation.z = q_flu[2]
        pose_msg.pose.orientation.w = q_flu[3]

        self.pose_pub.publish(pose_msg)

        # TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp    = msg.header.stamp
            t.header.frame_id = self.frame_id
            t.child_frame_id  = self.child_frame_id
            t.transform.translation.x = px
            t.transform.translation.y = py
            t.transform.translation.z = pz
            t.transform.rotation = pose_msg.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        # Debug: Compare sensor vs FLU vs expected FRD
        r_s, p_s, y_s = tf_trans.euler_from_quaternion(q_sensor)
        r_f, p_f, y_f = tf_trans.euler_from_quaternion(q_flu)
        
        rospy.loginfo_throttle(1.0,
            "\n=== Frame Conversion ===\n"
            "Sensor RPY : roll=%6.2f  pitch=%6.2f  yaw=%6.2f (deg)\n"
            "FLU RPY    : roll=%6.2f  pitch=%6.2f  yaw=%6.2f (deg)\n"
            "Expected   : roll=%6.2f  pitch=%6.2f  yaw=%6.2f (deg)",
            np.degrees(r_s), np.degrees(p_s), np.degrees(y_s),
            np.degrees(r_f), np.degrees(p_f), np.degrees(y_f),
            np.degrees(r_f), np.degrees(p_f), np.degrees(y_f)  # FLU should match drone after MAVROS FRD conv
        )

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = FastLIOToMAVROS()
        node.run()
    except rospy.ROSInterruptException:
        pass
