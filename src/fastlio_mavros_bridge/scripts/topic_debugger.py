#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Topic Debugger - Check what topics are available and receiving data
"""

import rospy
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State

class TopicDebugger:
    def __init__(self):
        rospy.init_node('topic_debugger', log_level=rospy.INFO)

        self.data_received = {
            '/Odometry': False,
            '/odometry': False,
            '/fast_lio/odom': False,
            '/mavros/vision_pose/pose': False,
            '/mavros/vision_pose/pose_cov': False,
            '/mavros/odometry/in': False,
            '/mavros/imu/data': False,
            '/mavros/state': False,
        }

        self.msg_counts = {}
        self.last_msgs = {}

        rospy.loginfo("="*70)
        rospy.loginfo("TOPIC DEBUGGER")
        rospy.loginfo("="*70)
        rospy.loginfo("Checking all possible topic names...")
        rospy.loginfo("")

        # Subscribe to many possible topic variations
        self.subs = []

        # FAST-LIO odometry (try multiple common names)
        possible_odom_topics = [
            '/Odometry',
            '/odometry',
            '/lio_sam/mapping/odometry',
            '/fast_lio/odom',
            '/lio_odom',
        ]

        for topic in possible_odom_topics:
            self.subs.append(rospy.Subscriber(topic, Odometry,
                lambda msg, t=topic: self.odom_callback(msg, t), queue_size=1))

        # MAVROS topics
        mavros_topics = [
            ('/mavros/vision_pose/pose', PoseStamped),
            ('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped),
            ('/mavros/odometry/in', Odometry),
            ('/mavros/odometry/out', Odometry),
            ('/mavros/imu/data', Imu),
            ('/mavros/state', State),
        ]

        for topic, msg_type in mavros_topics:
            self.subs.append(rospy.Subscriber(topic, msg_type,
                lambda msg, t=topic: self.generic_callback(msg, t), queue_size=1))

        # Timer to print status
        rospy.Timer(rospy.Duration(2.0), self.print_status)

        rospy.loginfo("Waiting for data on any topic...")
        rospy.loginfo("")

        rospy.spin()

    def odom_callback(self, msg, topic_name):
        self.data_received[topic_name] = True
        self.msg_counts[topic_name] = self.msg_counts.get(topic_name, 0) + 1
        self.last_msgs[topic_name] = msg

    def generic_callback(self, msg, topic_name):
        self.data_received[topic_name] = True
        self.msg_counts[topic_name] = self.msg_counts.get(topic_name, 0) + 1
        self.last_msgs[topic_name] = msg

    def print_status(self, event):
        rospy.loginfo("-"*70)
        rospy.loginfo("STATUS UPDATE (%s)", rospy.Time.now().to_sec())
        rospy.loginfo("-"*70)

        # Check FAST-LIO topics
        rospy.loginfo("\nFAST-LIO Topics:")
        found_fastlio = False
        for topic in ['/Odometry', '/odometry', '/fast_lio/odom', '/lio_sam/mapping/odometry']:
            if self.data_received.get(topic, False):
                found_fastlio = True
                count = self.msg_counts.get(topic, 0)
                msg = self.last_msgs.get(topic)
                if msg:
                    rospy.loginfo("  ✓ %s - %d msgs", topic, count)
                    rospy.loginfo("      Frame: %s", msg.header.frame_id)
                    rospy.loginfo("      Pos: X=%.2f Y=%.2f Z=%.2f",
                                msg.pose.pose.position.x,
                                msg.pose.pose.position.y,
                                msg.pose.pose.position.z)

        if not found_fastlio:
            rospy.logerr("  ✗ No FAST-LIO data on any topic!")
            rospy.logerr("    Expected: /Odometry")
            rospy.logerr("    Run: rostopic list | grep -i odometry")

        # Check MAVROS topics
        rospy.loginfo("\nMAVROS Topics:")

        # Vision pose
        if self.data_received.get('/mavros/vision_pose/pose_cov', False):
            msg = self.last_msgs.get('/mavros/vision_pose/pose_cov')
            rospy.loginfo("  ✓ /mavros/vision_pose/pose_cov - %d msgs",
                         self.msg_counts.get('/mavros/vision_pose/pose_cov', 0))
            if msg:
                rospy.loginfo("      Frame: %s", msg.header.frame_id)
                rospy.loginfo("      Pos: X=%.2f Y=%.2f Z=%.2f",
                            msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            msg.pose.pose.position.z)
                rospy.loginfo("      Cov: XX=%.4f YY=%.4f ZZ=%.4f",
                            msg.pose.covariance[0],
                            msg.pose.covariance[7],
                            msg.pose.covariance[14])
        else:
            rospy.logerr("  ✗ No /mavros/vision_pose/pose_cov data")
            rospy.logerr("    Bridge not publishing or wrong topic name")

        # FCU Odometry
        if self.data_received.get('/mavros/odometry/in', False):
            msg = self.last_msgs.get('/mavros/odometry/in')
            rospy.loginfo("  ✓ /mavros/odometry/in - %d msgs",
                         self.msg_counts.get('/mavros/odometry/in', 0))
            if msg:
                rospy.loginfo("      Pos: X=%.2f Y=%.2f Z=%.2f",
                            msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            msg.pose.pose.position.z)
        else:
            rospy.logwarn("  ⚠ No /mavros/odometry/in data")
            rospy.logwarn("    EKF may not be fusing vision data")
            rospy.logwarn("    Check: EK3_SRC1_POSXY=6, VISO_TYPE=1")

        # FCU State
        if self.data_received.get('/mavros/state', False):
            msg = self.last_msgs.get('/mavros/state')
            rospy.loginfo("  ✓ /mavros/state - Connected")
            rospy.loginfo("      Mode: %s | Armed: %s",
                         msg.mode, msg.armed)
        else:
            rospy.logerr("  ✗ No /mavros/state - MAVROS not connected to FCU!")

        # Recommendations
        rospy.loginfo("\n" + "-"*70)
        if not found_fastlio:
            rospy.logerr("ISSUE: FAST-LIO not publishing to expected topics")
            rospy.loginfo("FIX: Check FAST-LIO is running: rostopic hz /Odometry")
        elif not self.data_received.get('/mavros/vision_pose/pose_cov', False):
            rospy.logerr("ISSUE: Bridge not publishing to MAVROS")
            rospy.loginfo("FIX: Check bridge is running and frame_id is 'map_ned'")
        elif not self.data_received.get('/mavros/odometry/in', False):
            rospy.logerr("ISSUE: ArduPilot EKF not using vision data")
            rospy.loginfo("FIX: Set parameters:")
            rospy.loginfo("  EK3_SRC1_POSXY = 6")
            rospy.loginfo("  EK3_SRC1_POSZ = 6")
            rospy.loginfo("  EK3_SRC1_YAW = 6")
            rospy.loginfo("  VISO_TYPE = 1")
            rospy.loginfo("  AHRS_EKF_TYPE = 3")
        else:
            rospy.loginfo("✓ All data flowing correctly!")

        rospy.loginfo("-"*70 + "\n")


if __name__ == '__main__':
    try:
        TopicDebugger()
    except rospy.ROSInterruptException:
        pass
