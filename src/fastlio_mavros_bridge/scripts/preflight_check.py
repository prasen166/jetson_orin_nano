#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Pre-flight Check for FAST-LIO to MAVROS System
Run this before every flight to verify system health
"""

import rospy
import sys
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import State
import tf2_ros

class PreflightCheck:
    def __init__(self):
        rospy.init_node('preflight_check', log_level=rospy.INFO)

        self.checks_passed = 0
        self.checks_failed = 0
        self.checks_warnings = 0

        # Data storage
        self.fastlio_msg = None
        self.mavros_msg = None
        self.fc_state = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("="*70)
        rospy.loginfo("PREFLIGHT CHECK")
        rospy.loginfo("="*70)

        # Wait for data
        rospy.loginfo("\nCollecting data for 3 seconds...")
        rospy.Subscriber('/Odometry', Odometry, self.fastlio_callback)
        rospy.Subscriber('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped, self.mavros_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)

        rospy.sleep(3.0)
        self.run_checks()

    def fastlio_callback(self, msg):
        if self.fastlio_msg is None:
            self.fastlio_msg = msg

    def mavros_callback(self, msg):
        if self.mavros_msg is None:
            self.mavros_msg = msg

    def state_callback(self, msg):
        self.fc_state = msg

    def check_pass(self, msg):
        rospy.loginfo("  ✓ %s", msg)
        self.checks_passed += 1

    def check_fail(self, msg):
        rospy.loginfo("  ✗ %s", msg)
        self.checks_failed += 1

    def check_warn(self, msg):
        rospy.loginfo("  ⚠ %s", msg)
        self.checks_warnings += 1

    def run_checks(self):
        # Check 1: FAST-LIO data
        rospy.loginfo("\n[1] FAST-LIO Data Check:")
        if self.fastlio_msg:
            self.check_pass("FAST-LIO publishing to /Odometry")
            fl = self.fastlio_msg.pose.pose
            rospy.loginfo("      Position: X=%.2f Y=%.2f Z=%.2f",
                         fl.position.x, fl.position.y, fl.position.z)

            # Check if values are reasonable
            if abs(fl.position.x) > 100 or abs(fl.position.y) > 100:
                self.check_warn("Position far from origin - was SLAM reset?")
            if abs(fl.position.z) > 10:
                self.check_warn("Altitude high - check barometer calibration")
        else:
            self.check_fail("No FAST-LIO data! Check if FAST-LIO is running")

        # Check 2: MAVROS data
        rospy.loginfo("\n[2] MAVROS Vision Data Check:")
        if self.mavros_msg:
            self.check_pass("Bridge publishing to MAVROS")
            mv = self.mavros_msg.pose.pose
            rospy.loginfo("      Position: X=%.2f Y=%.2f Z=%.2f",
                         mv.position.x, mv.position.y, mv.position.z)

            # Check frame_id
            if self.mavros_msg.header.frame_id == "map_ned":
                self.check_pass("Frame ID is 'map_ned'")
            else:
                self.check_fail("Frame ID is '%s' but should be 'map_ned'!" %
                              self.mavros_msg.header.frame_id)

            # Check Z sign
            if self.fastlio_msg:
                fl = self.fastlio_msg.pose.pose
                if fl.position.z > 0 and mv.position.z > 0:
                    self.check_fail("Z sign is wrong! Both positive (NED Z should be negative)")
                elif fl.position.z > 0 and mv.position.z < 0:
                    self.check_pass("Z sign correct (ENU+ → NED-)")

            # Check conversion accuracy
            if self.fastlio_msg:
                fl = self.fastlio_msg.pose.pose
                exp_x = fl.position.y
                exp_y = fl.position.x
                exp_z = -fl.position.z

                err = math.sqrt(
                    (mv.position.x - exp_x)**2 +
                    (mv.position.y - exp_y)**2 +
                    (mv.position.z - exp_z)**2
                )

                if err < 0.01:
                    self.check_pass("Conversion accurate (error=%.4fm)" % err)
                else:
                    self.check_fail("Conversion error: %.4fm" % err)

            # Check covariance
            cov = self.mavros_msg.pose.covariance
            if cov[0] < 1.0 and cov[7] < 1.0 and cov[14] < 1.0:
                self.check_pass("Covariance reasonable")
            else:
                self.check_warn("High covariance - EKF will trust less")
        else:
            self.check_fail("No MAVROS vision data! Check bridge is running")

        # Check 3: TF Tree
        rospy.loginfo("\n[3] TF Tree Check:")
        try:
            # Get all frames
            all_frames = self.tf_buffer.all_frames_as_string()
            rospy.loginfo("      Available: %s", all_frames.replace('\n', ', ')[:100])

            # Check transforms
            transforms_to_check = [
                ("map_ned", "odom_ned"),
                ("odom", "camera_init"),
                ("map_ned", "base_link_frd"),
            ]

            for parent, child in transforms_to_check:
                try:
                    self.tf_buffer.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(0.5))
                    self.check_pass("TF %s -> %s exists" % (parent, child))
                except Exception as e:
                    self.check_fail("TF %s -> %s: %s" % (parent, child, str(e)[:40]))

        except Exception as e:
            self.check_warn("TF check failed: %s" % str(e))

        # Check 4: FCU State
        rospy.loginfo("\n[4] FCU State Check:")
        if self.fc_state:
            self.check_pass("Connected to FCU")
            rospy.loginfo("      Armed: %s | Mode: %s",
                         self.fc_state.armed, self.fc_state.mode)

            if self.fc_state.armed:
                self.check_warn("FCU is armed! Be careful!")

            # Check if in vision-capable mode
            if self.fc_state.mode in ["GUIDED", "LOITER", "GUIDED_NOGPS"]:
                self.check_pass("FCU in vision-capable mode (%s)" % self.fc_state.mode)
            else:
                self.check_warn("FCU mode is '%s' - need GUIDED/LOITER for vision" %
                              self.fc_state.mode)
        else:
            self.check_fail("No FCU state! Check MAVROS connection")

        # Summary
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("SUMMARY: %d passed, %d failed, %d warnings",
                     self.checks_passed, self.checks_failed, self.checks_warnings)

        if self.checks_failed > 0:
            rospy.loginfo("\n✗ PREFLIGHT CHECK FAILED")
            rospy.loginfo("\nFix the failed checks before flying!")
            rospy.loginfo("="*70)
            sys.exit(1)
        elif self.checks_warnings > 0:
            rospy.loginfo("\n⚠ PREFLIGHT CHECK PASSED WITH WARNINGS")
            rospy.loginfo("\nReview warnings before flying!")
            rospy.loginfo("="*70)
            sys.exit(0)
        else:
            rospy.loginfo("\n✓ PREFLIGHT CHECK PASSED")
            rospy.loginfo("\nSystem ready for flight!")
            rospy.loginfo("="*70)
            sys.exit(0)

if __name__ == '__main__':
    try:
        PreflightCheck()
    except rospy.ROSInterruptException:
        pass
