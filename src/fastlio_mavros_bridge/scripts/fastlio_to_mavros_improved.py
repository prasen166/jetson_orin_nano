#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from mavros_msgs.srv import StreamRate

pub_pose  = None
pub_twist = None
static_broadcaster = None  # ← declared at module level, no 'global' needed inside main

R_ENU_NED = np.array([
    [ 0,  1,  0],
    [ 1,  0,  0],
    [ 0,  0, -1]
], dtype=float)

def rotate_position(x, y, z):
    v = R_ENU_NED @ np.array([x, y, z])
    return v[0], v[1], v[2]

def rotate_quaternion_enu_to_ned(qx, qy, qz, qw):
    rx, ry, rz, rw = 0.5, 0.5, -0.5, 0.5
    nx = rw*qx + rx*qw + ry*qz - rz*qy
    ny = rw*qy - rx*qz + ry*qw + rz*qx
    nz = rw*qz + rx*qy - ry*qx + rz*qw
    nw = rw*qw - rx*qx - ry*qy - rz*qz
    return nx, ny, nz, nw

def odom_cb(msg):
    now = rospy.Time.now()

    px, py, pz = rotate_position(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z)

    qx, qy, qz, qw = rotate_quaternion_enu_to_ned(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)

    lvx, lvy, lvz = rotate_position(
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z)

    avx, avy, avz = rotate_position(
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z)

    pose_msg = PoseStamped()
    pose_msg.header.stamp    = now
    pose_msg.header.frame_id = "map_ned"
    pose_msg.pose.position.x = px
    pose_msg.pose.position.y = py
    pose_msg.pose.position.z = pz
    pose_msg.pose.orientation.x = qx
    pose_msg.pose.orientation.y = qy
    pose_msg.pose.orientation.z = qz
    pose_msg.pose.orientation.w = qw
    pub_pose.publish(pose_msg)

    twist_msg = TwistStamped()
    twist_msg.header.stamp    = now
    twist_msg.header.frame_id = "base_link_frd"
    twist_msg.twist.linear.x  = lvx
    twist_msg.twist.linear.y  = lvy
    twist_msg.twist.linear.z  = lvz
    twist_msg.twist.angular.x = avx
    twist_msg.twist.angular.y = avy
    twist_msg.twist.angular.z = avz
    pub_twist.publish(twist_msg)

def make_tf(parent, child, tx=0., ty=0., tz=0.,
            qx=0., qy=0., qz=0., qw=1.):
    t = TransformStamped()
    t.header.stamp    = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id  = child
    t.transform.translation.x = tx
    t.transform.translation.y = ty
    t.transform.translation.z = tz
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    return t

def set_stream_rate_once(rate_hz=200):
    try:
        rospy.wait_for_service("/mavros/set_stream_rate", timeout=5.0)
        set_rate_srv = rospy.ServiceProxy("/mavros/set_stream_rate", StreamRate)
        set_rate_srv(0, rate_hz, 1)
        rospy.loginfo(f"[fastlio_bridge] MAVROS stream rate → {rate_hz} Hz")
    except Exception as e:
        rospy.logwarn(f"[fastlio_bridge] Stream rate set failed: {e}")

if __name__ == "__main__":
    rospy.init_node("fastlio_bridge")

    # Assign to module-level variables directly — no 'global' keyword needed here
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    pub_pose  = rospy.Publisher("/mavros/vision_pose/pose",
                                PoseStamped, queue_size=10)
    pub_twist = rospy.Publisher("/mavros/vision_speed/speed_twist",
                                TwistStamped, queue_size=10)

    odom_topic = rospy.get_param("~odom_topic", "/Odometry")
    rospy.Subscriber(odom_topic, Odometry, odom_cb)
    rospy.loginfo(f"[fastlio_bridge] Subscribed to {odom_topic}")

    rospy.sleep(1.0)

    static_broadcaster.sendTransform([
        make_tf("map", "camera_init")
    ])
    rospy.loginfo("[fastlio_bridge] Static TF published: map → camera_init (identity)")

    rospy.sleep(1.0)
    set_stream_rate_once(200)

    rospy.spin()
