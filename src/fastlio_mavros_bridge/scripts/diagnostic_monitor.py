#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
FAST-LIO to MAVROS Diagnostic Monitor
Compares FAST-LIO output with MAVROS input to detect conversion errors
"""

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
import tf
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class DiagnosticMonitor:
    def __init__(self):
        rospy.init_node('diagnostic_monitor', log_level=rospy.INFO)

        # Data storage
        self.last_fastlio_msg = None
        self.last_mavros_pose = None
        self.last_mavros_pose_cov = None
        self.last_fcu_odom = None
        self.last_imu = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Statistics
        self.fastlio_count = 0
        self.mavros_count = 0
        self.error_count = 0
        self.position_diffs = []
        self.yaw_diffs = []

        # Subscribers
        rospy.Subscriber('/Odometry', Odometry, self.fastlio_callback)
        rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.mavros_pose_callback)
        rospy.Subscriber('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped, self.mavros_pose_cov_callback)
        rospy.Subscriber('/mavros/odometry/in', Odometry, self.fcu_odom_callback)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)

        # Timer for diagnostics
        rospy.Timer(rospy.Duration(2.0), self.print_diagnostics)

        rospy.loginfo("="*60)
        rospy.loginfo("DIAGNOSTIC MONITOR STARTED")
        rospy.loginfo("Comparing FAST-LIO -> MAVROS -> FCU data flow")
        rospy.loginfo("="*60)

        rospy.spin()

    def fastlio_callback(self, msg):
        """Store FAST-LIO output (ENU frame)"""
        self.last_fastlio_msg = msg
        self.fastlio_count += 1

    def mavros_pose_callback(self, msg):
        """Store MAVROS pose input"""
        self.last_mavros_pose = msg
        self.mavros_count += 1

    def mavros_pose_cov_callback(self, msg):
        """Store MAVROS pose with covariance"""
        self.last_mavros_pose_cov = msg

    def fcu_odom_callback(self, msg):
        """Store FCU odometry output"""
        self.last_fcu_odom = msg

    def imu_callback(self, msg):
        """Store IMU data"""
        self.last_imu = msg

    def quat_to_yaw(self, q):
        """Convert quaternion to yaw angle (radians)"""
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def print_diagnostics(self, event):
        """Print comprehensive diagnostic information"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("DIAGNOSTIC REPORT - %s", rospy.Time.now().to_sec())
        rospy.loginfo("="*70)

        # 1. FAST-LIO Output (ENU)
        if self.last_fastlio_msg:
            fl = self.last_fastlio_msg.pose.pose
            fl_yaw = self.quat_to_yaw(fl.orientation)
            rospy.loginfo("\n[1] FAST-LIO OUTPUT (/Odometry) - ENU FRAME:")
            rospy.loginfo("    Position:  X=%.3f  Y=%.3f  Z=%.3f",
                         fl.position.x, fl.position.y, fl.position.z)
            rospy.loginfo("    Yaw:       %.2f deg (ENU frame)", math.degrees(fl_yaw))
            rospy.loginfo("    Frame:     %s", self.last_fastlio_msg.header.frame_id)

            # Check if values are reasonable
            issues = []
            if abs(fl.position.z) > 50:
                issues.append("Z very high - check altitude")
            if abs(fl.position.x) > 1000 or abs(fl.position.y) > 1000:
                issues.append("Position very far from origin")
            if issues:
                rospy.logwarn("    ISSUES: %s", " | ".join(issues))
        else:
            rospy.logerr("[1] NO FAST-LIO DATA! Check /Odometry topic")

        # 2. MAVROS Input (should be NED)
        if self.last_mavros_pose_cov:
            mp = self.last_mavros_pose_cov.pose.pose
            mp_yaw = self.quat_to_yaw(mp.orientation)
            rospy.loginfo("\n[2] MAVROS INPUT (/mavros/vision_pose/pose_cov):")
            rospy.loginfo("    Position:  X=%.3f  Y=%.3f  Z=%.3f",
                         mp.position.x, mp.position.y, mp.position.z)
            rospy.loginfo("    Yaw:       %.2f deg", math.degrees(mp_yaw))
            rospy.loginfo("    Frame:     %s", self.last_mavros_pose_cov.header.frame_id)

            # Check frame_id
            if self.last_mavros_pose_cov.header.frame_id != "map_ned":
                rospy.logerr("    ERROR: Frame is '%s' but should be 'map_ned'!",
                           self.last_mavros_pose_cov.header.frame_id)

            # Check covariance
            cov = self.last_mavros_pose_cov.pose.covariance
            rospy.loginfo("    Covariance: XX=%.4f YY=%.4f ZZ=%.4f",
                         cov[0], cov[7], cov[14])

            # Compare with FAST-LIO (expected conversion)
            if self.last_fastlio_msg:
                fl = self.last_fastlio_msg.pose.pose

                # Expected NED values from ENU
                expected_x = fl.position.y      # North = ENU Y
                expected_y = fl.position.x      # East = ENU X
                expected_z = -fl.position.z     # Down = -ENU Z

                dx = mp.position.x - expected_x
                dy = mp.position.y - expected_y
                dz = mp.position.z - expected_z

                rospy.loginfo("\n[3] CONVERSION CHECK (ENU -> NED):")
                rospy.loginfo("    Expected:  X=%.3f  Y=%.3f  Z=%.3f",
                             expected_x, expected_y, expected_z)
                rospy.loginfo("    Actual:    X=%.3f  Y=%.3f  Z=%.3f",
                             mp.position.x, mp.position.y, mp.position.z)
                rospy.loginfo("    Error:     dX=%.4f dY=%.4f dZ=%.4f", dx, dy, dz)

                pos_error = math.sqrt(dx*dx + dy*dy + dz*dz)
                if pos_error > 0.01:
                    rospy.logerr("    ERROR: Position mismatch! Error=%.4f", pos_error)
                    self.error_count += 1
                else:
                    rospy.loginfo("    OK: Position conversion correct")

                # Check Z sign (critical!)
                if mp.position.z > 0 and fl.position.z > 0:
                    rospy.logerr("    ERROR: Z is positive in NED! Should be negative when above ground!")
                    rospy.logerr("           FAST-LIO Z=%.3f (Up+) -> MAVROS Z=%.3f (Down-)",
                               fl.position.z, mp.position.z)
        else:
            rospy.logerr("[2] NO MAVROS DATA! Check /mavros/vision_pose/pose_cov topic")

        # 4. FCU Output
        if self.last_fcu_odom:
            fc = self.last_fcu_odom.pose.pose
            fc_yaw = self.quat_to_yaw(fc.orientation)
            rospy.loginfo("\n[4] FCU OUTPUT (/mavros/odometry/in):")
            rospy.loginfo("    Position:  X=%.3f  Y=%.3f  Z=%.3f",
                         fc.position.x, fc.position.y, fc.position.z)
            rospy.loginfo("    Yaw:       %.2f deg", math.degrees(fc_yaw))

            if self.last_mavros_pose_cov:
                mp = self.last_mavros_pose_cov.pose.pose
                dx = fc.position.x - mp.position.x
                dy = fc.position.y - mp.position.y
                dz = fc.position.z - mp.position.z
                fcu_error = math.sqrt(dx*dx + dy*dy + dz*dz)
                rospy.loginfo("    vs MAVROS: dX=%.3f dY=%.3f dZ=%.3f (error=%.3f)",
                             dx, dy, dz, fcu_error)
                if fcu_error > 1.0:
                    rospy.logerr("    WARNING: FCU position diverged from MAVROS input!")
        else:
            rospy.logwarn("[4] NO FCU ODOMETRY - EKF may not be using vision data")

        # 5. TF Tree check
        rospy.loginfo("\n[5] TF TREE CHECK:")
        try:
            # Get all frame names from TF buffer
            frame_list = self.tf_buffer.all_frames_as_string()
            rospy.loginfo("    Available frames: %s", frame_list.replace('\n', ', '))

            # Check specific transforms
            required_transforms = [
                ("map_ned", "odom_ned"),
                ("odom_ned", "odom"),
                ("odom", "camera_init"),
                ("odom", "base_link"),
                ("map_ned", "base_link")
            ]

            for parent, child in required_transforms:
                try:
                    self.tf_buffer.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(0.1))
                    rospy.loginfo("    %s -> %s: OK", parent, child)
                except tf2_ros.LookupException:
                    rospy.logwarn("    %s -> %s: MISSING", parent, child)
                except Exception as e:
                    rospy.logwarn("    %s -> %s: %s", parent, child, str(e)[:30])

        except Exception as e:
            rospy.logwarn("    TF check failed: %s", str(e))

        # 6. IMU check
        if self.last_imu:
            imu = self.last_imu
            rospy.loginfo("\n[6] IMU DATA:")
            rospy.loginfo("    Angular Vel: X=%.3f Y=%.3f Z=%.3f",
                         imu.angular_velocity.x,
                         imu.angular_velocity.y,
                         imu.angular_velocity.z)
            rospy.loginfo("    Linear Accel: X=%.3f Y=%.3f Z=%.3f",
                         imu.linear_acceleration.x,
                         imu.linear_acceleration.y,
                         imu.linear_acceleration.z)

            # Check if IMU is responding (Z accel should be ~9.81 when level)
            total_accel = math.sqrt(imu.linear_acceleration.x**2 +
                                   imu.linear_acceleration.y**2 +
                                   imu.linear_acceleration.z**2)
            if abs(total_accel - 9.81) > 2.0:
                rospy.logwarn("    WARNING: Total acceleration=%.2f (expected ~9.81)", total_accel)

        # Summary
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("SUMMARY: Messages received - FAST-LIO:%d MAVROS:%d Errors:%d",
                     self.fastlio_count, self.mavros_count, self.error_count)

        if self.error_count > 0:
            rospy.logerr("CRITICAL: %d conversion errors detected!", self.error_count)
        rospy.loginfo("="*70 + "\n")


if __name__ == '__main__':
    try:
        DiagnosticMonitor()
    except rospy.ROSInterruptException:
        pass
