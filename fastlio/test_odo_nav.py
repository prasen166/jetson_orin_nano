#!/usr/bin/env python3

from pymodbus.client import ModbusTcpClient as ModbusClient
import rospy
import math
import time
import numpy as np
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Int32

# =====================================================
# MODBUS CONFIG
# =====================================================
PLC_HOST = '192.168.140.2'
PLC_PORT = 502
UNIT = 0x1

client = ModbusClient(PLC_HOST, PLC_PORT)
client.connect()

# =====================================================
# GLOBAL STATE
# =====================================================
current_x = 0.0
current_y = 0.0
heading = 0.0

previous_velocity = 0.0
current_velocity = 0.0

roi_flag = False
roi_obs_dist = 50.0
avoid_status = False
brake_applied = False

# =====================================================
# PATH
# =====================================================
WAYPOINT_FILE = "/home/tihan/fastlio/waypoints_fastlio.txt"
path = []

with open(WAYPOINT_FILE) as f:
    for line in f:
        x, y = map(float, line.strip().split(','))
        path.append((x, y))

# =====================================================
# INITIAL LOCALIZATION
# =====================================================
T_INIT = None
lio_pose = None
ndt_pose = None
initialized = False

wp = 0
LOOKAHEAD = 4

# =====================================================
# HELPERS
# =====================================================
def pose_to_mat(p):
    q = p.orientation
    t = p.position
    T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    T[0, 3] = t.x
    T[1, 3] = t.y
    T[2, 3] = t.z
    return T

def normalize_angle(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a

# =====================================================
# MODBUS HELPERS
# =====================================================
def set_steer(angle):
    direction = angle < 0
    angle = abs(angle)
    client.write_coils(20, direction, unit=UNIT)
    client.write_registers(400, int(angle), unit=UNIT)

def accelerate(value):
    client.write_registers(500, int(value), unit=UNIT)

def set_drive_mode(forward=False, reverse=False):
    client.write_coils(502, forward, unit=UNIT)
    client.write_coils(504, reverse, unit=UNIT)

def apply_brake():
    global brake_applied
    client.write_coil(3, True, unit=UNIT)
    time.sleep(0.3)
    client.write_coil(3, False, unit=UNIT)
    brake_applied = True

def remove_brake():
    global brake_applied
    client.write_coil(4, True, unit=UNIT)
    brake_applied = False

# =====================================================
# PATH PROJECTION
# =====================================================
def nearest_path_index(x, y, start_idx):
    best = start_idx
    best_dist = 1e9
    for i in range(start_idx, min(start_idx + 50, len(path))):
        d = math.hypot(path[i][0] - x, path[i][1] - y)
        if d < best_dist:
            best_dist = d
            best = i
    return best

# =====================================================
# CALLBACKS
# =====================================================
def fastlio_cb(msg):
    global lio_pose
    lio_pose = msg

def ndt_cb(msg):
    global ndt_pose
    ndt_pose = msg

def roi_dist_cb(msg):
    global roi_obs_dist
    roi_obs_dist = msg.data

def roi_flag_cb(msg):
    global roi_flag
    roi_flag = msg.data

def avoid_cb(msg):
    global avoid_status
    avoid_status = msg.data

# =====================================================
# INITIAL OFFSET COMPUTATION
# =====================================================
def compute_initial_offset():
    global T_INIT
    T_lio = pose_to_mat(lio_pose.pose.pose)
    T_ndt = pose_to_mat(ndt_pose.pose)
    T_INIT = T_ndt @ np.linalg.inv(T_lio)
    rospy.loginfo("Initial localization offset computed")

# =====================================================
# NAVIGATION CALLBACK
# =====================================================
def navigation_cb(msg):
    global current_x, current_y, heading
    global wp, previous_velocity, initialized

    if not initialized:
        return

    # ---- APPLY OFFSET ----
    T_lio = pose_to_mat(msg.pose.pose)
    T = T_INIT @ T_lio

    current_x = T[0, 3]
    current_y = T[1, 3]
    yaw = tf.transformations.euler_from_matrix(T)[2]
    heading = math.degrees(yaw)

    # ---- PUBLISH CORRECTED POSE ----
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"
    pose_msg.pose.position.x = current_x
    pose_msg.pose.position.y = current_y
    pose_msg.pose.position.z = 0.0
    q = tf.transformations.quaternion_from_euler(0, 0, yaw)
    pose_msg.pose.orientation.x = q[0]
    pose_msg.pose.orientation.y = q[1]
    pose_msg.pose.orientation.z = q[2]
    pose_msg.pose.orientation.w = q[3]
    pub_corrected_pose.publish(pose_msg)

    # ---- PROJECT TO PATH ----
    wp = nearest_path_index(current_x, current_y, wp)

    lookahead_idx = min(wp + LOOKAHEAD, len(path) - 1)
    pub_ld_index.publish(lookahead_idx)

    target_x, target_y = path[lookahead_idx]
    off_x = target_x - current_x
    off_y = target_y - current_y

    bearing = math.degrees(math.atan2(off_y, off_x))
    bearing_diff = normalize_angle(bearing - heading)

    # ---- OBSTACLE LOGIC ----
    if roi_flag:
        vel_obs = max(0, 25 - 25 * math.exp(-0.4 * (roi_obs_dist - 8)))
    else:
        vel_obs = 12

    if roi_flag and roi_obs_dist < 5.0:
        accelerate(0)
        if not brake_applied:
            apply_brake()
        return
    else:
        if brake_applied:
            remove_brake()

    target_vel = min(vel_obs, 12)
    current_velocity = min(100, previous_velocity + 0.25)
    previous_velocity = current_velocity

    set_drive_mode(forward=True)
    accelerate(current_velocity)

    steer_output = int(
        160 * math.atan2(
            2 * 3.5 * math.sin(math.radians(bearing_diff)),
            LOOKAHEAD
        )
    )
    set_steer(steer_output)

# =====================================================
# NODE START
# =====================================================
rospy.init_node("fastlio_teach_repeat_navigation")

pub_corrected_pose = rospy.Publisher(
    "/fastlio_corrected_pose", PoseStamped, queue_size=10
)
pub_ld_index = rospy.Publisher(
    "/current_lookahead_index", Int32, queue_size=10
)

rospy.Subscriber("/Odometry", Odometry, fastlio_cb)
rospy.Subscriber("/ndt_pose", PoseStamped, ndt_cb)
rospy.Subscriber("/roi_min_distance", Float32, roi_dist_cb)
rospy.Subscriber("/roi_obstacle", Bool, roi_flag_cb)
rospy.Subscriber("/avoidance_status", Bool, avoid_cb)

rospy.loginfo("Waiting for FAST-LIO odometry and NDT pose...")
while lio_pose is None or ndt_pose is None:
    rospy.sleep(0.1)

compute_initial_offset()
initialized = True
rospy.loginfo("Initialization done. Navigation started.")

rospy.Subscriber("/Odometry", Odometry, navigation_cb)
rospy.spin()

