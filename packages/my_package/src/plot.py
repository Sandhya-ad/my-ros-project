#!/usr/bin/env python3

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry

# Initialize position and heading
x, y, theta = 0, 0, 0
x_vals, y_vals = [x], [y]

def compute_trajectory():
    global x, y, theta

    bag = rosbag.Bag('move.bag')

    print("Reading velocity and angular velocity from bag file...")

    prev_time = None

    for topic, msg, t in bag.read_messages(topics=['/csc22907/deadreckoning_node/odom']):
        v = msg.twist.twist.linear.x   # Linear velocity (m/s)
        omega = msg.twist.twist.angular.z  # Angular velocity (rad/s)

        current_time = t.to_sec()

        # Compute dt (time difference)
        if prev_time is None:
            prev_time = current_time
            continue
        dt = current_time - prev_time
        prev_time = current_time

        # Apply kinematic equations
        theta += omega * dt
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt

        x_vals.append(x)
        y_vals.append(y)

        print(f"Time: {current_time:.2f}, X: {x:.3f}, Y: {y:.3f}, Theta: {theta:.3f}")

    bag.close()

def plot_trajectory():
    compute_trajectory()

    plt.figure(figsize=(8, 6))
    plt.plot(x_vals, y_vals, marker='o', linestyle='-', markersize=3, label="Duckiebot Trajectory")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Duckiebot Trajectory Using Kinematic Equations")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == '__main__':
    plot_trajectory()
