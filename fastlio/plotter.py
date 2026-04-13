#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import matplotlib

# Use safe backend (important for compatibility)
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

# Data storage
time_data = []

x_data, y_data, z_data = [], [], []
roll_data, pitch_data, yaw_data = [], [], []

start_time = None

def callback(msg):
    global start_time

    t = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

    if start_time is None:
        start_time = t

    t = t - start_time

    # Position
    pos = msg.pose.pose.position
    x_data.append(pos.x)
    y_data.append(pos.y)
    z_data.append(pos.z)

    # Orientation
    q = msg.pose.pose.orientation
    quaternion = [q.x, q.y, q.z, q.w]
    roll, pitch, yaw = euler_from_quaternion(quaternion)

    roll_data.append(roll)
    pitch_data.append(pitch)
    yaw_data.append(yaw)

    time_data.append(t)


def live_plot():
    plt.ion()

    fig, axs = plt.subplots(2, 1, figsize=(10, 8))

    # ---- Initialize lines once (IMPORTANT FIX) ----
    line_x, = axs[0].plot([], [], label='X')
    line_y, = axs[0].plot([], [], label='Y')
    line_z, = axs[0].plot([], [], label='Z')

    line_r, = axs[1].plot([], [], label='Roll')
    line_p, = axs[1].plot([], [], label='Pitch')
    line_yaw, = axs[1].plot([], [], label='Yaw')

    # Labels & grid (set once)
    axs[0].set_title("Position vs Time")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("Position")
    axs[0].legend()
    axs[0].grid()

    axs[1].set_title("Orientation vs Time")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Radians")
    axs[1].legend()
    axs[1].grid()

    while not rospy.is_shutdown():

        # ---- Update data instead of clearing ----
        line_x.set_data(time_data, x_data)
        line_y.set_data(time_data, y_data)
        line_z.set_data(time_data, z_data)

        line_r.set_data(time_data, roll_data)
        line_p.set_data(time_data, pitch_data)
        line_yaw.set_data(time_data, yaw_data)

        # Auto-scale axes
        for ax in axs:
            ax.relim()
            ax.autoscale_view()

        plt.pause(0.05)


def listener():
    rospy.init_node('odom_live_plot_with_orientation', anonymous=True)
    rospy.Subscriber('/Odometry', Odometry, callback)

    live_plot()


if __name__ == '__main__':
    listener()
