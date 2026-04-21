#!/usr/bin/env python3
"""
FAST-LIO Live Euler Angle Subscriber
Subscribes to /Odometry (nav_msgs/Odometry) published by FAST-LIO
and continuously prints roll, pitch, yaw in degrees and radians.
"""

import math
import rospy
from nav_msgs.msg import Odometry


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to Euler angles (ZYX convention). Returns radians."""
    # Roll (X-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y-axis)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    # Yaw (Z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def odom_callback(msg):
    q = msg.pose.pose.orientation
    roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

    seq  = msg.header.seq
    secs = msg.header.stamp.secs
    nsecs= msg.header.stamp.nsecs

    rospy.loginfo(
        "\n"
        "─────────────────────────────────────────────────\n"
        f"  Seq: {seq}  |  Stamp: {secs}.{nsecs:09d}\n"
        "─────────────────────────────────────────────────\n"
        f"  Quaternion : x={q.x:.6f}  y={q.y:.6f}  z={q.z:.6f}  w={q.w:.6f}\n"
        f"  Roll       : {roll:.6f} rad  |  {math.degrees(roll):.4f} deg\n"
        f"  Pitch      : {pitch:.6f} rad  |  {math.degrees(pitch):.4f} deg\n"
        f"  Yaw        : {yaw:.6f} rad  |  {math.degrees(yaw):.4f} deg\n"
    )


def main():
    rospy.init_node("fastlio_euler_printer", anonymous=True)

    # FAST-LIO publishes odometry on /Odometry by default.
    # Change the topic below if your setup uses a different name.
    topic = rospy.get_param("~topic", "/Odometry")

    rospy.loginfo(f"[fastlio_euler_printer] Subscribing to: {topic}")
    rospy.Subscriber(topic, Odometry, odom_callback, queue_size=10)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
