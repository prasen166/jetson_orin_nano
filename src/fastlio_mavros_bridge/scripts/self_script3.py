#!/usr/bin/env python3

#####################################################
##        ROS Odometry to MAVLink Bridge           ##
#####################################################
# Subscribes to a ROS /odometry (nav_msgs/Odometry) topic
# and forwards pose + velocity data as MAVLink messages.
#
# Install required packages:
#   pip3 install pymavlink
#   pip3 install apscheduler
#   pip3 install pyserial
#   pip3 install transformations
#   ROS: nav_msgs, geometry_msgs (standard ROS packages)
#
# Usage:
#   python3 ros_odometry_to_mavlink.py [--connect /dev/ttyUSB0] [--baudrate 921600]
#                                       [--odometry_topic /odometry]
#                                       [--camera_orientation 0]
#                                       [--debug_enable 1]

import sys
sys.path.append("/usr/local/lib/")

import os
os.environ["MAVLINK20"] = "1"

import numpy as np
import tf.transformations as tf
import math as m
import time
import argparse
import threading
import signal

from time import sleep
from apscheduler.schedulers.background import BackgroundScheduler
from pymavlink import mavutil

# ROS imports
import rospy
from nav_msgs.msg import Odometry


# Replacement of the standard print() function to flush the output
def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()


#######################################
# Parameters
#######################################

# Default configurations for connection to the FCU
connection_string_default   = '/dev/ttyUSB0'
connection_baudrate_default = 921600
connection_timeout_sec_default = 5

# ROS topic to subscribe to (nav_msgs/Odometry)
odometry_topic_default = '/odometry'

# Camera/sensor orientation relative to vehicle body (NED convention)
#   0: Forward, sensor X forward
#   1: Downfacing
camera_orientation_default = 0

# MAVLink message enable flags and rates
enable_msg_vision_position_estimate      = True
vision_position_estimate_msg_hz_default  = 30.0

enable_msg_vision_position_delta         = False
vision_position_delta_msg_hz_default     = 30.0

enable_msg_vision_speed_estimate         = True
vision_speed_estimate_msg_hz_default     = 30.0

enable_update_tracking_confidence_to_gcs       = True
update_tracking_confidence_to_gcs_hz_default   = 1.0

# EKF home / GPS origin (used when no GPS is present)
enable_auto_set_ekf_home = False
home_lat = 151269321
home_lon = 16624301
home_alt = 163000

# Body offset from IMU/CoG to sensor origin (meters)
body_offset_enabled = 0
body_offset_x = 0.0
body_offset_y = 0.0
body_offset_z = 0.0

# Global scale factor applied to position data
scale_factor = 1.0

# Enable compass-aligned yaw (align north = 0 deg)
compass_enabled = 0

# Confidence string labels (index matches tracker_confidence 0–3)
pose_data_confidence_level = ('FAILED', 'Low', 'Medium', 'High')

# Thread synchronization
lock = threading.Lock()
mavlink_thread_should_exit = False

# Default exit code (failure); graceful SIGTERM sets it to 0
exit_code = 1


#######################################
# Global variables
#######################################

# MAVLink connection handle (set in main)
conn = None

# Pose / velocity matrices in aero (NED) reference frame
H_aeroRef_aeroBody = None
V_aeroRef_aeroBody = None

# Compass heading for north-alignment
heading_north_yaw = None

# Latest odometry message received from ROS
latest_odom = None

# Derived confidence (0–100 %) – set from odometry covariance or fixed
current_confidence_level = 100.0   # default: full confidence
current_time_us = 0

# Increment on pose jumps / relocalization
reset_counter = 1


#######################################
# Argument parsing
#######################################

parser = argparse.ArgumentParser(description='ROS Odometry → MAVLink bridge')
parser.add_argument('--connect',
                    help="Vehicle connection string (default: %s)" % connection_string_default)
parser.add_argument('--baudrate', type=float,
                    help="Vehicle connection baudrate (default: %d)" % connection_baudrate_default)
parser.add_argument('--odometry_topic',
                    help="ROS Odometry topic to subscribe (default: %s)" % odometry_topic_default)
parser.add_argument('--vision_position_estimate_msg_hz', type=float,
                    help="VISION_POSITION_ESTIMATE publish rate (Hz)")
parser.add_argument('--vision_position_delta_msg_hz', type=float,
                    help="VISION_POSITION_DELTA publish rate (Hz)")
parser.add_argument('--vision_speed_estimate_msg_hz', type=float,
                    help="VISION_SPEED_ESTIMATE publish rate (Hz)")
parser.add_argument('--camera_orientation', type=int,
                    help="Sensor orientation (0=forward, 1=downfacing)")
parser.add_argument('--debug_enable', type=int,
                    help="Enable debug messages (1=on)")

args = parser.parse_args()

connection_string   = args.connect          or connection_string_default
connection_baudrate = args.baudrate         or connection_baudrate_default
odometry_topic      = args.odometry_topic   or odometry_topic_default

vision_position_estimate_msg_hz = args.vision_position_estimate_msg_hz or vision_position_estimate_msg_hz_default
vision_position_delta_msg_hz    = args.vision_position_delta_msg_hz    or vision_position_delta_msg_hz_default
vision_speed_estimate_msg_hz    = args.vision_speed_estimate_msg_hz    or vision_speed_estimate_msg_hz_default

camera_orientation = args.camera_orientation if args.camera_orientation is not None else camera_orientation_default
debug_enable       = 1 if args.debug_enable else 0

progress("INFO: connection_string          = %s" % connection_string)
progress("INFO: connection_baudrate        = %s" % connection_baudrate)
progress("INFO: odometry_topic            = %s" % odometry_topic)
progress("INFO: vision_position_estimate  = %.1f Hz" % vision_position_estimate_msg_hz)
progress("INFO: vision_speed_estimate     = %.1f Hz" % vision_speed_estimate_msg_hz)
progress("INFO: camera_orientation        = %d" % camera_orientation)

if debug_enable:
    np.set_printoptions(precision=4, suppress=True)
    progress("INFO: Debug messages enabled.")

# -----------------------------------------------------------------------
# Build the fixed rotation matrices based on camera/sensor orientation.
# These convert the odometry frame (assumed ROS REP-105 FLU/ENU or body)
# into the aeronautic NED frame expected by ArduPilot.
#
# ROS convention (REP-103): x=forward, y=left, z=up  (ENU-body)
# NED convention:           x=forward, y=right, z=down
#
# The transformation below maps ROS body → NED body for a forward-facing
# sensor.  Adjust H_rosBody_aeroBody for your specific mounting.
# -----------------------------------------------------------------------

if camera_orientation == 0:     # Forward-facing sensor
    # ROS ENU reference → NED reference
    H_aeroRef_rosRef   = np.array([[0, 1, 0, 0],
                                    [1, 0, 0, 0],
                                    [0, 0,-1, 0],
                                    [0, 0, 0, 1]])
    # ROS body (FLU) → NED body (FRD)
    H_rosBody_aeroBody = np.array([[1, 0, 0, 0],
                                    [0,-1, 0, 0],
                                    [0, 0,-1, 0],
                                    [0, 0, 0, 1]])
elif camera_orientation == 1:   # Downfacing sensor
    H_aeroRef_rosRef   = np.array([[0, 1, 0, 0],
                                    [1, 0, 0, 0],
                                    [0, 0,-1, 0],
                                    [0, 0, 0, 1]])
    H_rosBody_aeroBody = np.array([[0, 1, 0, 0],
                                    [1, 0, 0, 0],
                                    [0, 0,-1, 0],
                                    [0, 0, 0, 1]])
else:                            # Default = forward-facing
    H_aeroRef_rosRef   = np.array([[0, 1, 0, 0],
                                    [1, 0, 0, 0],
                                    [0, 0,-1, 0],
                                    [0, 0, 0, 1]])
    H_rosBody_aeroBody = np.array([[1, 0, 0, 0],
                                    [0,-1, 0, 0],
                                    [0, 0,-1, 0],
                                    [0, 0, 0, 1]])


#######################################
# MAVLink helper functions
#######################################

def mavlink_loop(conn, callbacks):
    """Background thread: reads MAVLink messages and dispatches callbacks."""
    interesting_messages = list(callbacks.keys())
    while not mavlink_thread_should_exit:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            0, 0, 0)
        msg = conn.recv_match(type=interesting_messages, timeout=1, blocking=True)
        if msg is None:
            continue
        callbacks[msg.get_type()](msg)


def send_vision_position_estimate_message():
    """Send VISION_POSITION_ESTIMATE MAVLink message."""
    global current_time_us, H_aeroRef_aeroBody, reset_counter, latest_odom
    with lock:
        if H_aeroRef_aeroBody is None:
            return

        rpy_rad = np.array(tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))

        # Derive covariance from odometry pose covariance if available,
        # otherwise use a fixed medium-confidence value.
        if latest_odom is not None and any(v != 0 for v in latest_odom.pose.covariance):
            cov_pose  = latest_odom.pose.covariance[0]   # xx
            cov_twist = latest_odom.twist.covariance[0]  # vx
        else:
            cov_pose  = 0.1
            cov_twist = 0.1

        covariance = np.array([cov_pose, 0, 0, 0, 0, 0,
                                          cov_pose, 0, 0, 0, 0,
                                                    cov_pose, 0, 0, 0,
                                                              cov_twist, 0, 0,
                                                                         cov_twist, 0,
                                                                                    cov_twist])

        conn.mav.vision_position_estimate_send(
            current_time_us,
            H_aeroRef_aeroBody[0][3],   # X (North)
            H_aeroRef_aeroBody[1][3],   # Y (East)
            H_aeroRef_aeroBody[2][3],   # Z (Down)
            rpy_rad[0],                 # Roll
            rpy_rad[1],                 # Pitch
            rpy_rad[2],                 # Yaw
            covariance,
            reset_counter
        )


def send_vision_position_delta_message():
    """Send VISION_POSITION_DELTA MAVLink message."""
    global current_time_us, current_confidence_level, H_aeroRef_aeroBody
    with lock:
        if H_aeroRef_aeroBody is None:
            return

        H_prev = send_vision_position_delta_message.H_aeroRef_PrevAeroBody
        H_delta = np.linalg.inv(H_prev).dot(H_aeroRef_aeroBody)

        delta_time_us    = current_time_us - send_vision_position_delta_message.prev_time_us
        delta_position_m = [H_delta[0][3], H_delta[1][3], H_delta[2][3]]
        delta_angle_rad  = np.array(tf.euler_from_matrix(H_delta, 'sxyz'))

        conn.mav.vision_position_delta_send(
            current_time_us,
            delta_time_us,
            delta_angle_rad,
            delta_position_m,
            current_confidence_level
        )

        send_vision_position_delta_message.H_aeroRef_PrevAeroBody = H_aeroRef_aeroBody
        send_vision_position_delta_message.prev_time_us = current_time_us


def send_vision_speed_estimate_message():
    """Send VISION_SPEED_ESTIMATE MAVLink message."""
    global current_time_us, V_aeroRef_aeroBody, reset_counter, latest_odom
    with lock:
        if V_aeroRef_aeroBody is None:
            return

        if latest_odom is not None and any(v != 0 for v in latest_odom.twist.covariance):
            cov_pose = latest_odom.twist.covariance[0]
        else:
            cov_pose = 0.1

        covariance = np.array([cov_pose, 0,        0,
                                0,       cov_pose,  0,
                                0,       0,         cov_pose])

        conn.mav.vision_speed_estimate_send(
            current_time_us,
            V_aeroRef_aeroBody[0][3],   # Vx (North)
            V_aeroRef_aeroBody[1][3],   # Vy (East)
            V_aeroRef_aeroBody[2][3],   # Vz (Down)
            covariance,
            reset_counter
        )


def update_tracking_confidence_to_gcs():
    """Send confidence level status text to GCS (rate-limited)."""
    global current_confidence_level
    prev = update_tracking_confidence_to_gcs.prev_confidence_level
    if current_confidence_level != prev:
        # Map 0-100 to nearest label
        idx = min(3, int(current_confidence_level / 34))
        send_msg_to_gcs('Tracking confidence: %s (%.0f%%)' % (pose_data_confidence_level[idx], current_confidence_level))
        update_tracking_confidence_to_gcs.prev_confidence_level = current_confidence_level


def send_msg_to_gcs(text_to_be_sent):
    """Send a STATUSTEXT MAVLink message to the GCS."""
    text_msg = 'ODOM: ' + text_to_be_sent
    conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text_msg.encode())
    progress("INFO: %s" % text_to_be_sent)


def set_default_global_origin():
    conn.mav.set_gps_global_origin_send(1, home_lat, home_lon, home_alt)


def set_default_home_position():
    conn.mav.set_home_position_send(
        1, home_lat, home_lon, home_alt,
        0, 0, 0,
        [1, 0, 0, 0],
        0, 0, 1
    )


def att_msg_callback(value):
    """Callback for ATTITUDE messages; captures initial compass yaw."""
    global heading_north_yaw
    if heading_north_yaw is None:
        heading_north_yaw = value.yaw
        progress("INFO: Initial ATTITUDE yaw = %.2f deg" % m.degrees(heading_north_yaw))


def increment_reset_counter():
    global reset_counter
    reset_counter = 1 if reset_counter >= 255 else reset_counter + 1


#######################################
# ROS Odometry callback
#######################################

def odometry_callback(msg):
    """
    Receives nav_msgs/Odometry messages from ROS and converts pose + twist
    into NED matrices ready for MAVLink transmission.

    Assumptions:
      - msg.pose.pose.position   is in the ROS ENU/map frame
      - msg.pose.pose.orientation is a quaternion (x, y, z, w) – ROS convention
      - msg.twist.twist.linear   is the velocity in the child (body) frame
    """
    global H_aeroRef_aeroBody, V_aeroRef_aeroBody
    global current_time_us, current_confidence_level, latest_odom
    global reset_counter

    with lock:
        latest_odom     = msg
        current_time_us = int(round(time.time() * 1000000))

        # ---- Position & orientation ----------------------------------------
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation   # ROS quaternion: x y z w

        # Build 4x4 homogeneous matrix in ROS reference frame.
        # transformations.py uses [w, x, y, z] ordering.
        H_rosRef_rosBody = tf.quaternion_matrix([ori.w, ori.x, ori.y, ori.z])
        H_rosRef_rosBody[0][3] = pos.x * scale_factor
        H_rosRef_rosBody[1][3] = pos.y * scale_factor
        H_rosRef_rosBody[2][3] = pos.z * scale_factor

        # Transform into aeronautic (NED) reference frame
        H_aeroRef_aeroBody = H_aeroRef_rosRef.dot(
            H_rosRef_rosBody.dot(H_rosBody_aeroBody)
        )

        # ---- Velocity -------------------------------------------------------
        vel = msg.twist.twist.linear   # velocity in body (child) frame

        # Express velocity in the ROS reference frame first, then convert to NED.
        V_rosRef = tf.quaternion_matrix([1, 0, 0, 0])
        V_rosRef[0][3] = vel.x
        V_rosRef[1][3] = vel.y
        V_rosRef[2][3] = vel.z

        V_aeroRef_aeroBody = H_aeroRef_rosRef.dot(V_rosRef)

        # ---- Body offset correction ----------------------------------------
        if body_offset_enabled == 1:
            H_body_camera        = tf.euler_matrix(0, 0, 0, 'sxyz')
            H_body_camera[0][3]  = body_offset_x
            H_body_camera[1][3]  = body_offset_y
            H_body_camera[2][3]  = body_offset_z
            H_camera_body        = np.linalg.inv(H_body_camera)
            H_aeroRef_aeroBody   = H_body_camera.dot(
                H_aeroRef_aeroBody.dot(H_camera_body)
            )

        # ---- Compass north-alignment ----------------------------------------
        if compass_enabled == 1 and heading_north_yaw is not None:
            H_aeroRef_aeroBody = H_aeroRef_aeroBody.dot(
                tf.euler_matrix(0, 0, heading_north_yaw, 'sxyz')
            )

        # ---- Confidence from pose covariance --------------------------------
        # Use the diagonal variance of the position covariance as a proxy.
        # covariance is a 6×6 row-major array; [0]=xx, [7]=yy, [14]=zz
        cov = msg.pose.covariance
        mean_var = (abs(cov[0]) + abs(cov[7]) + abs(cov[14])) / 3.0
        if mean_var < 0.01:
            current_confidence_level = 100.0
        elif mean_var < 0.1:
            current_confidence_level = 66.0
        elif mean_var < 1.0:
            current_confidence_level = 33.0
        else:
            current_confidence_level = 0.0

        # ---- Debug output ---------------------------------------------------
        if debug_enable == 1:
            os.system('clear')
            progress("DEBUG: ROS pos  xyz : {}".format(
                np.array([pos.x, pos.y, pos.z])))
            progress("DEBUG: NED pos  xyz : {}".format(
                np.array(tf.translation_from_matrix(H_aeroRef_aeroBody))))
            progress("DEBUG: NED RPY [deg]: {}".format(
                np.array(tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz')) * 180 / m.pi))
            progress("DEBUG: NED vel  xyz : [{:.3f}  {:.3f}  {:.3f}]".format(
                V_aeroRef_aeroBody[0][3],
                V_aeroRef_aeroBody[1][3],
                V_aeroRef_aeroBody[2][3]))
            progress("DEBUG: Confidence   : {:.0f} %%".format(current_confidence_level))


#######################################
# Main
#######################################

try:
    progress("INFO: Starting Vehicle communications")
    conn = mavutil.mavlink_connection(
        connection_string,
        autoreconnect  = True,
        source_system  = 1,
        source_component = 93,
        baud           = connection_baudrate,
        force_connected= True,
    )

    mavlink_callbacks = {
        'ATTITUDE': att_msg_callback,
    }

    mavlink_thread = threading.Thread(
        target=mavlink_loop,
        args=(conn, mavlink_callbacks)
    )
    mavlink_thread.start()

    # ---- ROS node setup -----------------------------------------------------
    progress("INFO: Initialising ROS node...")
    rospy.init_node('odometry_to_mavlink', anonymous=True, disable_signals=True)

    progress("INFO: Subscribing to ROS topic: %s" % odometry_topic)
    rospy.Subscriber(odometry_topic, Odometry, odometry_callback, queue_size=10)
    send_msg_to_gcs('Subscribed to ROS odometry topic: ' + odometry_topic)

    # ---- Scheduled MAVLink publishers ---------------------------------------
    sched = BackgroundScheduler()

    if enable_msg_vision_position_estimate:
        sched.add_job(
            send_vision_position_estimate_message,
            'interval',
            seconds=1.0 / vision_position_estimate_msg_hz
        )

    if enable_msg_vision_position_delta:
        sched.add_job(
            send_vision_position_delta_message,
            'interval',
            seconds=1.0 / vision_position_delta_msg_hz
        )
        send_vision_position_delta_message.H_aeroRef_PrevAeroBody = tf.quaternion_matrix([1, 0, 0, 0])
        send_vision_position_delta_message.prev_time_us = int(round(time.time() * 1000000))

    if enable_msg_vision_speed_estimate:
        sched.add_job(
            send_vision_speed_estimate_message,
            'interval',
            seconds=1.0 / vision_speed_estimate_msg_hz
        )

    if enable_update_tracking_confidence_to_gcs:
        sched.add_job(
            update_tracking_confidence_to_gcs,
            'interval',
            seconds=1.0 / update_tracking_confidence_to_gcs_hz_default
        )
        update_tracking_confidence_to_gcs.prev_confidence_level = -1

    sched.start()

    # ---- Optional EKF home init --------------------------------------------
    if enable_auto_set_ekf_home:
        send_msg_to_gcs('Setting EKF home to default GPS location')
        set_default_global_origin()
        set_default_home_position()

    # ---- Graceful shutdown handlers ----------------------------------------
    main_loop_should_quit = False

    def sigint_handler(sig, frame):
        global main_loop_should_quit
        main_loop_should_quit = True

    def sigterm_handler(sig, frame):
        global main_loop_should_quit, exit_code
        main_loop_should_quit = True
        exit_code = 0

    signal.signal(signal.SIGINT,  sigint_handler)
    signal.signal(signal.SIGTERM, sigterm_handler)

    if compass_enabled == 1:
        time.sleep(1)

    send_msg_to_gcs('Bridge running – forwarding odometry to FCU')
    progress("INFO: Waiting for odometry data on %s ..." % odometry_topic)

    # ---- Main loop: keep alive while ROS and MAVLink are running -----------
    while not main_loop_should_quit and not rospy.is_shutdown():
        time.sleep(0.1)

except Exception as e:
    progress("ERROR: %s" % str(e))
    import traceback
    traceback.print_exc()

finally:
    progress("INFO: Shutting down...")
    mavlink_thread_should_exit = True
    mavlink_thread.join()
    conn.close()
    progress("INFO: MAVLink connection closed.")
    try:
        rospy.signal_shutdown("Bridge exiting")
    except Exception:
        pass
    sys.exit(exit_code)
