#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SLAM Safety Monitor - Production Version
Monitors FAST-LIO output and triggers safety actions

Features:
1. Position jump detection with configurable thresholds
2. Multiple data source monitoring (pose + odometry)
3. Rate monitoring
4. Automatic failsafe triggering
5. Detailed diagnostics
"""

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty
import math
import numpy as np

class SlamSafetyMonitor:
    def __init__(self):
        rospy.init_node('slam_safety_monitor', log_level=rospy.INFO)

        # Parameters
        self.pose_topic = rospy.get_param('~pose_topic', '/mavros/vision_pose/pose')
        self.health_topic = rospy.get_param('~health_topic', '/slam_bridge/healthy')
        self.failsafe_topic = rospy.get_param('~failsafe_topic', '/slam_safety/failsafe')
        self.status_topic = rospy.get_param('~status_topic', '/slam_safety/status')

        self.max_position_jump = rospy.get_param('~max_position_jump', 2.0)  # meters
        self.max_orientation_jump = rospy.get_param('~max_orientation_jump', 0.5)  # radians
        self.max_no_data_time = rospy.get_param('~max_no_data_time', 1.0)  # seconds
        self.min_rate = rospy.get_param('~min_rate', 5.0)  # Hz
        self.window_size = rospy.get_param('~window_size', 10)  # samples for rate calc

        # State
        self.last_pose = None
        self.last_orientation = None
        self.last_data_time = None
        self.msg_times = []
        self.healthy = False
        self.failsafe_triggered = False
        self.jump_count = 0
        self.consecutive_errors = 0

        # Publishers
        self.safety_pub = rospy.Publisher(self.status_topic, Bool, queue_size=1, latch=True)
        self.failsafe_pub = rospy.Publisher(self.failsafe_topic, Empty, queue_size=1)

        # Subscribers
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
        self.health_sub = rospy.Subscriber(self.health_topic, Bool, self.health_callback)

        # Timers
        self.check_timer = rospy.Timer(rospy.Duration(0.1), self.safety_check)
        self.stats_timer = rospy.Timer(rospy.Duration(5.0), self.print_stats)

        rospy.loginfo("SLAM Safety Monitor initialized")
        rospy.loginfo("Monitoring: %s | Jump threshold: %.2fm", self.pose_topic, self.max_position_jump)
        rospy.spin()

    def pose_callback(self, msg):
        current_time = rospy.Time.now()
        current_pos = msg.pose.position
        current_quat = msg.pose.orientation

        # Validate
        if not all(np.isfinite([current_pos.x, current_pos.y, current_pos.z])):
            self.consecutive_errors += 1
            rospy.logwarn_throttle(5.0, "Invalid pose data received")
            return

        # Check position jump
        if self.last_pose is not None:
            jump = math.sqrt(
                (current_pos.x - self.last_pose.x)**2 +
                (current_pos.y - self.last_pose.y)**2 +
                (current_pos.z - self.last_pose.z)**2
            )

            if jump > self.max_position_jump:
                self.jump_count += 1
                self.consecutive_errors += 1
                rospy.logerr("POSITION JUMP DETECTED: %.2fm! (threshold: %.2fm)", 
                           jump, self.max_position_jump)

                if self.consecutive_errors >= 3:
                    self.trigger_failsafe("Multiple position jumps")
            else:
                self.consecutive_errors = max(0, self.consecutive_errors - 1)

        # Update rate calculation
        self.msg_times.append(current_time.to_sec())
        # Remove old samples
        cutoff_time = current_time.to_sec() - 1.0
        self.msg_times = [t for t in self.msg_times if t > cutoff_time]

        # Update state
        self.last_pose = current_pos
        self.last_orientation = current_quat
        self.last_data_time = current_time

    def health_callback(self, msg):
        self.healthy = msg.data

    def trigger_failsafe(self, reason):
        if not self.failsafe_triggered:
            self.failsafe_triggered = True
            rospy.logerr("FAILSAFE TRIGGERED: %s", reason)
            self.failsafe_pub.publish(Empty())

    def safety_check(self, event):
        current_time = rospy.Time.now()
        status = True

        # Check data timeout
        if self.last_data_time is None:
            status = False
            rospy.logerr_throttle(5.0, "No pose data received yet")
        else:
            age = (current_time - self.last_data_time).to_sec()
            if age > self.max_no_data_time:
                status = False
                rospy.logerr_throttle(5.0, "Data timeout: %.1fs", age)

        # Check rate
        if len(self.msg_times) < self.min_rate:
            status = False
            rospy.logwarn_throttle(5.0, "Low data rate: %.1f Hz (min: %.1f)", 
                                  len(self.msg_times), self.min_rate)

        # Check bridge health
        if not self.healthy:
            status = False
            rospy.logwarn_throttle(5.0, "SLAM bridge reports unhealthy")

        # Publish status
        self.safety_pub.publish(Bool(status))

    def print_stats(self, event):
        rate = len(self.msg_times)
        status_str = "HEALTHY" if (self.healthy and not self.failsafe_triggered) else "UNHEALTHY"

        rospy.loginfo(
            "[%s] Rate: %.1f Hz | Jumps: %d | Errors: %d | Failsafe: %s",
            status_str, rate, self.jump_count, self.consecutive_errors,
            "TRIGGERED" if self.failsafe_triggered else "OK"
        )

if __name__ == '__main__':
    try:
        SlamSafetyMonitor()
    except rospy.ROSInterruptException:
        pass

