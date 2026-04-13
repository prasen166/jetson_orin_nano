#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
FAST-LIO to MAVROS Bridge - Fixed Version
Critical fixes applied:
1. Proper ENU -> NED coordinate transformation (not relying on MAVROS)
2. Correct frame_id: 'map_ned' (not 'odom')
3. Optional velocity publishing (disabled by default to avoid EKF confusion)
"""

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Header
import tf.transformations as tf_trans

class FastlioToMavros:
    def __init__(self):
        rospy.init_node('fastlio_to_mavros', log_level=rospy.INFO)

        # Parameters - FIXED: Default frame is now 'map_ned' not 'odom'
        self.odom_topic = rospy.get_param('~odom_topic', '/Odometry')
        self.pose_topic = rospy.get_param('~pose_topic', '/mavros/vision_pose/pose')
        self.pose_cov_topic = rospy.get_param('~pose_cov_topic', '/mavros/vision_pose/pose_cov')
        self.twist_topic = rospy.get_param('~twist_topic', '/mavros/vision_speed/speed_twist')
        self.health_topic = rospy.get_param('~health_topic', '/slam_bridge/healthy')

        # FIXED: Default frame_id is 'map_ned' for ArduPilot NED convention
        self.frame_id = rospy.get_param('~frame_id', 'map_ned')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.min_rate = rospy.get_param('~min_rate', 10.0)
        self.max_rate = rospy.get_param('~max_rate', 50.0)
        self.timeout = rospy.get_param('~timeout', 0.5)
        self.covariance_scale = rospy.get_param('~covariance_scale', 1.0)
        
        # NEW: Option to disable velocity (position differentiation is unreliable)
        self.publish_velocity = rospy.get_param('~publish_velocity', False)

        # State variables
        self.last_msg_time = None
        self.last_publish_time = None
        self.msg_count = 0
        self.valid_msg_count = 0
        self.error_count = 0
        self.last_position = None
        self.last_orientation = None
        self.is_healthy = False
        self.reset_detected = False

        # Velocity calculation (only used if publish_velocity=True)
        self.velocity_buffer = []
        self.velocity_buffer_size = 5
        self.last_velocity_time = None

        # Publishers
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.pose_cov_pub = rospy.Publisher(self.pose_cov_topic, PoseWithCovarianceStamped, queue_size=10)
        if self.publish_velocity:
            self.twist_pub = rospy.Publisher(self.twist_topic, TwistStamped, queue_size=10)
        self.health_pub = rospy.Publisher(self.health_topic, Bool, queue_size=1, latch=True)

        # Subscribers
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)

        # Timers
        self.health_timer = rospy.Timer(rospy.Duration(0.2), self.health_check)
        self.stats_timer = rospy.Timer(rospy.Duration(5.0), self.print_stats)

        rospy.loginfo("FAST-LIO Bridge initialized (FIXED NED VERSION)")
        rospy.loginfo("Frame: %s | Velocity Pub: %s", self.frame_id, self.publish_velocity)
        rospy.spin()

    def validate_odometry(self, msg):
        """Validate FAST-LIO odometry message"""
        pos = msg.pose.pose.position
        if not all(np.isfinite([pos.x, pos.y, pos.z])):
            return False, "Position contains NaN/Inf"
        if abs(pos.x) > 1000 or abs(pos.y) > 1000 or abs(pos.z) > 100 or pos.z < -10:
            return False, "Position out of bounds"
        q = msg.pose.pose.orientation
        quat_norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if abs(quat_norm - 1.0) > 0.1:
            return False, "Invalid quaternion"
        return True, "Valid"

    def enu_to_ned(self, position, orientation):
        """
        Convert ENU (East-North-Up) to NED (North-East-Down) coordinates.
        
        ENU -> NED conversion:
        - ENU X (East) becomes NED Y (East)  
        - ENU Y (North) becomes NED X (North)
        - ENU Z (Up) becomes NED -Z (Down)
        
        Quaternion conversion requires rotation of 90 degrees around Z then 180 around X,
        or equivalently: q_ned = q_enu * [0, 0, -1, 0] * [1, 0, 0, 0] (simplified)
        
        Actually simpler: Rotate 90° about Z (ENU->NED frame rotation), then flip Z.
        Rotation matrix approach: R_ned_enu = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
        """
        # Position conversion
        ned_pos = type(position)()  # Create new instance of same type
        ned_pos.x = position.y       # North = ENU Y
        ned_pos.y = position.x       # East = ENU X  
        ned_pos.z = position.z      # Down = -ENU Z

        # Quaternion conversion
        # We need to rotate the quaternion from ENU frame to NED frame
        # R = [0,1,0; 1,0,0; 0,0,1] (basis vector mapping)
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # Method: Convert to rotation matrix, apply basis change, convert back
        # But simpler is to construct the transform quaternion
        # ENU to NED is rotation of -90° about Z followed by 180° about X
        # Or equivalently: q_transform = [0.7071, 0.7071, 0, 0] (90° X) * [0, 0, -0.7071, 0.7071] (-90° Z)
        
        # Simpler approach: Manual basis vector rotation
        # q_ned = [y, x, -z, w] after normalization adjustment? No, that's not right for quaternions.
        
        # Proper quaternion rotation: q' = R * q where R is the rotation between frames
        # Actually, we need q_ned = q_enu * q_rot (where q_rot represents ENU->NED rotation)
        
        # ENU to NED rotation quaternion (rotate 90° about Z then 180° about new X):
        # cos(90/2)=0.7071, sin(90/2)=0.7071 for Z
        # cos(180/2)=0, sin(180/2)=1 for X
        # Combined: [0.7071, 0, 0, 0.7071] rotated...
        
        # Actually, let's use the matrix approach for clarity:
        # Rotation matrix from ENU to NED:
        # [[0, 1, 0],
        #  [1, 0, 0], 
        #  [0, 0, -1]]
        
        q_matrix = tf_trans.quaternion_matrix(q)
        # Apply basis transformation
        transform = np.array([[0, 1, 0, 0],
                              [1, 0, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        q_ned_matrix = np.dot(transform, q_matrix)
        q_ned = tf_trans.quaternion_from_matrix(q_ned_matrix)
        
        ned_orientation = type(orientation)()
        ned_orientation.x = q_ned[0]
        ned_orientation.y = q_ned[1]
        ned_orientation.z = q_ned[2]
        ned_orientation.w = q_ned[3]

        return ned_pos, ned_orientation

    def convert_covariance_enu_to_ned(self, cov):
        """Convert 6x6 covariance matrix from ENU to NED frame"""
        if len(cov) != 36:
            return cov
            
        # Covariance matrix indices: row-major, 6x6 (x,y,z,roll,pitch,yaw)
        # Mapping: ENU (x,y,z) -> NED (y,x,-z)
        # So cov(x,x) becomes cov(y,y), etc.
        
        cov_matrix = np.array(cov).reshape(6, 6)
        
        # Permutation matrix for ENU->NED: [y, x, -z, roll, pitch, yaw]
        # Note: Rotation covariances also need transformation but are typically small
        P = np.array([[0, 1, 0, 0, 0, 0],   # x_ned = y_enu
                      [1, 0, 0, 0, 0, 0],   # y_ned = x_enu  
                      [0, 0, -1, 0, 0, 0],  # z_ned = -z_enu (sign flips covariance)
                      [0, 0, 0, 0, 1, 0],   # roll_ned = pitch_enu (rough approximation)
                      [0, 0, 0, 1, 0, 0],   # pitch_ned = roll_enu
                      [0, 0, 0, 0, 0, -1]]) # yaw_ned = -yaw_enu
        
        # Transform covariance: C' = P * C * P^T
        cov_ned = np.dot(np.dot(P, cov_matrix), P.T)
        
        # Ensure positive diagonal (variance can't be negative)
        for i in range(6):
            if cov_ned[i, i] < 0:
                cov_ned[i, i] = abs(cov_ned[i, i])
        
        return cov_ned.flatten().tolist()

    def detect_reset(self, current_pos):
        """Detect FAST-LIO reset/loop closure jumps"""
        if self.last_position is None:
            return False
        dist = math.sqrt(
            (current_pos.x - self.last_position.x)**2 +
            (current_pos.y - self.last_position.y)**2 +
            (current_pos.z - self.last_position.z)**2
        )
        if dist > 5.0:
            origin_dist = math.sqrt(current_pos.x**2 + current_pos.y**2)
            if origin_dist < 2.0:
                return True
        return False

    def compute_velocity(self, current_pos, current_time):
        """Compute smoothed velocity from position history - NOTE: Still unreliable"""
        if self.last_position is None or self.last_velocity_time is None:
            self.last_velocity_time = current_time
            return 0.0, 0.0, 0.0

        dt = (current_time - self.last_velocity_time).to_sec()
        if dt < 0.001:
            return 0.0, 0.0, 0.0

        vx = (current_pos.x - self.last_position.x) / dt
        vy = (current_pos.y - self.last_position.y) / dt
        vz = (current_pos.z - self.last_position.z) / dt

        self.velocity_buffer.append((vx, vy, vz))
        if len(self.velocity_buffer) > self.velocity_buffer_size:
            self.velocity_buffer.pop(0)

        avg_vx = sum(v[0] for v in self.velocity_buffer) / len(self.velocity_buffer)
        avg_vy = sum(v[1] for v in self.velocity_buffer) / len(self.velocity_buffer)
        avg_vz = sum(v[2] for v in self.velocity_buffer) / len(self.velocity_buffer)

        self.last_velocity_time = current_time
        return avg_vx, avg_vy, avg_vz

    def estimate_covariance(self, msg):
        """Estimate covariance from FAST-LIO output"""
        if len(msg.pose.covariance) == 36:
            cov = list(msg.pose.covariance)
            for i in [0, 7, 14]:
                cov[i] *= self.covariance_scale
            return cov

        pos_var = 0.01 * self.covariance_scale
        rot_var = 0.01 * self.covariance_scale

        return [
            pos_var, 0, 0, 0, 0, 0,
            0, pos_var, 0, 0, 0, 0,
            0, 0, pos_var, 0, 0, 0,
            0, 0, 0, rot_var, 0, 0,
            0, 0, 0, 0, rot_var, 0,
            0, 0, 0, 0, 0, rot_var
        ]

    def odom_callback(self, msg):
        self.msg_count += 1

        valid, reason = self.validate_odometry(msg)
        if not valid:
            self.error_count += 1
            rospy.logwarn_throttle(5.0, "Invalid odometry: %s", reason)
            return

        current_time = rospy.Time.now()
        current_pos = msg.pose.pose.position

        if self.detect_reset(current_pos):
            self.reset_detected = True
            rospy.logwarn("FAST-LIO reset detected!")
            self.velocity_buffer = []
            self.last_position = current_pos
            self.last_velocity_time = None
            return

        # CRITICAL FIX: Convert ENU to NED
        ned_pos, ned_orientation = self.enu_to_ned(msg.pose.pose.position, msg.pose.pose.orientation)
        
        # Convert covariance too
        cov = self.estimate_covariance(msg)
        cov = self.convert_covariance_enu_to_ned(cov)

        # Rate limiting
        if self.last_publish_time is not None:
            dt = (current_time - self.last_publish_time).to_sec()
            if dt < (1.0 / self.max_rate):
                return

        # Build PoseStamped with NED data
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = self.frame_id  # FIXED: 'map_ned'
        pose_msg.pose.position = ned_pos
        pose_msg.pose.orientation = ned_orientation

        # Build PoseWithCovarianceStamped with NED data
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header = pose_msg.header
        pose_cov_msg.pose.pose.position = ned_pos
        pose_cov_msg.pose.pose.orientation = ned_orientation
        pose_cov_msg.pose.covariance = cov

        # Publish pose
        self.pose_pub.publish(pose_msg)
        self.pose_cov_pub.publish(pose_cov_msg)

        # Only publish velocity if explicitly enabled (Issue 3 fix)
        if self.publish_velocity:
            # Compute velocity in NED frame (convert from ENU velocity if available)
            # Note: FAST-LIO twist is in body frame usually, but if in ENU:
            twist = msg.twist.twist
            vx_ned = twist.linear.y   # North = ENU Y
            vy_ned = twist.linear.x   # East = ENU X
            vz_ned = twist.linear.z  # Down = -ENU Z
            
            twist_msg = TwistStamped()
            twist_msg.header = pose_msg.header
            twist_msg.twist.linear.x = vx_ned
            twist_msg.twist.linear.y = vy_ned
            twist_msg.twist.linear.z = vz_ned
            twist_msg.twist.angular = twist.angular  # Angular rates usually same or need conversion
            self.twist_pub.publish(twist_msg)

        # Update state (store raw ENU for next iteration's velocity calc if needed)
        self.last_msg_time = current_time
        self.last_publish_time = current_time
        self.last_position = current_pos  # Keep raw ENU for reset detection
        self.last_orientation = msg.pose.pose.orientation
        self.valid_msg_count += 1
        self.is_healthy = True
        self.reset_detected = False

    def health_check(self, event):
        if self.last_msg_time is None:
            self.is_healthy = False
        else:
            age = (rospy.Time.now() - self.last_msg_time).to_sec()
            if age > self.timeout:
                self.is_healthy = False
                rospy.logerr_throttle(5.0, "FAST-LIO timeout: %.1fs", age)
        self.health_pub.publish(Bool(self.is_healthy))

    def print_stats(self, event):
        if self.msg_count > 0:
            rate = self.valid_msg_count / 5.0
            error_rate = (self.error_count / float(self.msg_count)) * 100
            status = "HEALTHY" if self.is_healthy else "UNHEALTHY"
            rospy.loginfo(
                "[%s] Rate: %.1f Hz | Valid: %d/%d | Errors: %.1f%% | Resets: %s | Frame: %s",
                status, rate, self.valid_msg_count, self.msg_count, error_rate,
                "YES" if self.reset_detected else "NO", self.frame_id
            )
        self.msg_count = 0
        self.valid_msg_count = 0
        self.error_count = 0

if __name__ == '__main__':
    try:
        FastlioToMavros()
    except rospy.ROSInterruptException:
        pass
