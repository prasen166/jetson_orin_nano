#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry

# ================= USER PARAMETERS =================
WAYPOINT_DISTANCE = 1.0   # meters
OUTPUT_FILE = "/home/tihan/fastlio/waypoints_fastlio.txt"
ODOM_TOPIC = "/Odometry"  # FAST-LIO odometry topic
# ==================================================

last_point = None
total_points = 0


def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def odom_callback(msg):
    global last_point, total_points

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    current_point = (x, y)

    # First point → always save
    if last_point is None:
        last_point = current_point
        with open(OUTPUT_FILE, "w") as f:
            f.write(f"{x},{y}\n")
        total_points += 1
        rospy.loginfo(f"Saved first waypoint: {x:.3f}, {y:.3f}")
        return

    # Save only if moved >= 1 meter
    if distance(last_point, current_point) >= WAYPOINT_DISTANCE:
        with open(OUTPUT_FILE, "a") as f:
            f.write(f"{x},{y}\n")
        last_point = current_point
        total_points += 1
        rospy.loginfo(f"Waypoint {total_points} saved: {x:.3f}, {y:.3f}")


def main():
    rospy.init_node("fastlio_waypoint_collector", anonymous=True)

    rospy.loginfo("FAST-LIO waypoint collection started")
    rospy.loginfo(f"Saving waypoints every {WAYPOINT_DISTANCE} meter(s)")
    rospy.loginfo(f"Output file: {OUTPUT_FILE}")

    rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback, queue_size=1)

    rospy.spin()


if __name__ == "__main__":
    main()

