#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CSV Logger for FAST-LIO to MAVROS Debugging
Logs all relevant data to CSV for analysis in Excel/Python/MATLAB
"""

import rospy
import csv
import os
from datetime import datetime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import math

class CSVLogger:
    def __init__(self):
        rospy.init_node('csv_logger', log_level=rospy.INFO)

        # Create output file
        home_dir = os.path.expanduser("~")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(home_dir, "fastlio_mavros_log_%s.csv" % timestamp)

        # Open CSV file
        self.csvfile = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.csvfile)

        # Write header
        self.writer.writerow([
            # Timestamp
            'timestamp',
            # FAST-LIO ENU
            'fl_enu_x', 'fl_enu_y', 'fl_enu_z',
            'fl_enu_qx', 'fl_enu_qy', 'fl_enu_qz', 'fl_enu_qw',
            'fl_enu_roll', 'fl_enu_pitch', 'fl_enu_yaw',
            'fl_enu_vx', 'fl_enu_vy', 'fl_enu_vz',
            # MAVROS NED (what we send)
            'mv_ned_x', 'mv_ned_y', 'mv_ned_z',
            'mv_ned_qx', 'mv_ned_qy', 'mv_ned_qz', 'mv_ned_qw',
            'mv_ned_roll', 'mv_ned_pitch', 'mv_ned_yaw',
            # Expected NED (what MAVROS should be)
            'exp_ned_x', 'exp_ned_y', 'exp_ned_z',
            'exp_ned_yaw',
            # Errors
            'err_x', 'err_y', 'err_z', 'err_pos',
            # FCU Output (if available)
            'fcu_x', 'fcu_y', 'fcu_z', 'fcu_yaw',
            # Status
            'frame_id', 'cov_xx', 'cov_yy', 'cov_zz'
        ])

        self.csvfile.flush()

        # Data storage
        self.last_fastlio = None
        self.last_mavros = None
        self.last_fcu = None

        # Subscribers
        rospy.Subscriber('/Odometry', Odometry, self.fastlio_callback)
        rospy.Subscriber('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped, self.mavros_callback)
        rospy.Subscriber('/mavros/odometry/in', Odometry, self.fcu_callback)

        # Log timer (10 Hz)
        rospy.Timer(rospy.Duration(0.1), self.log_callback)

        rospy.loginfo("CSV Logger started: %s", self.filename)
        rospy.spin()

    def quat_to_euler(self, q):
        """Convert quaternion to roll, pitch, yaw"""
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def fastlio_callback(self, msg):
        self.last_fastlio = msg

    def mavros_callback(self, msg):
        self.last_mavros = msg

    def fcu_callback(self, msg):
        self.last_fcu = msg

    def log_callback(self, event):
        if not self.last_fastlio or not self.last_mavros:
            return

        fl = self.last_fastlio.pose.pose
        mv = self.last_mavros.pose.pose

        # Convert quaternions to Euler
        fl_roll, fl_pitch, fl_yaw = self.quat_to_euler(fl.orientation)
        mv_roll, mv_pitch, mv_yaw = self.quat_to_euler(mv.orientation)

        # Expected NED from FAST-LIO ENU
        exp_x = fl.position.y      # North = ENU Y
        exp_y = fl.position.x      # East = ENU X
        exp_z = -fl.position.z     # Down = -ENU Z
        exp_yaw = fl_yaw           # Yaw stays same (but check!)

        # Errors
        err_x = mv.position.x - exp_x
        err_y = mv.position.y - exp_y
        err_z = mv.position.z - exp_z
        err_pos = math.sqrt(err_x**2 + err_y**2 + err_z**2)

        # FCU data (if available)
        fcu_x = fcu_y = fcu_z = fcu_yaw = 0.0
        if self.last_fcu:
            fcu = self.last_fcu.pose.pose
            fcu_x = fcu.position.x
            fcu_y = fcu.position.y
            fcu_z = fcu.position.z
            _, _, fcu_yaw = self.quat_to_euler(fcu.orientation)

        # Write row
        self.writer.writerow([
            rospy.Time.now().to_sec(),
            # FAST-LIO ENU
            fl.position.x, fl.position.y, fl.position.z,
            fl.orientation.x, fl.orientation.y, fl.orientation.z, fl.orientation.w,
            fl_roll, fl_pitch, fl_yaw,
            self.last_fastlio.twist.twist.linear.x,
            self.last_fastlio.twist.twist.linear.y,
            self.last_fastlio.twist.twist.linear.z,
            # MAVROS NED
            mv.position.x, mv.position.y, mv.position.z,
            mv.orientation.x, mv.orientation.y, mv.orientation.z, mv.orientation.w,
            mv_roll, mv_pitch, mv_yaw,
            # Expected
            exp_x, exp_y, exp_z, exp_yaw,
            # Errors
            err_x, err_y, err_z, err_pos,
            # FCU
            fcu_x, fcu_y, fcu_z, fcu_yaw,
            # Status
            self.last_mavros.header.frame_id,
            self.last_mavros.pose.covariance[0],
            self.last_mavros.pose.covariance[7],
            self.last_mavros.pose.covariance[14]
        ])

        self.csvfile.flush()

        # Print warnings
        if err_pos > 0.01:
            rospy.logwarn_throttle(1.0, "CONVERSION ERROR: %.4f m", err_pos)

    def shutdown(self):
        self.csvfile.close()
        rospy.loginfo("CSV log saved to: %s", self.filename)

if __name__ == '__main__':
    logger = None
    try:
        logger = CSVLogger()
    except rospy.ROSInterruptException:
        pass
    finally:
        if logger:
            logger.shutdown()
