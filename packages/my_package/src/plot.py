import rosbag
import numpy as np
import matplotlib.pyplot as plt
import math

# Constants
WHEEL_BASE = 0.2  # Adjust this based on your robot's real dimensions
radius = 3.1  # Wheel radius

# Initialize position and heading
x, y, theta = 0.0, 0.0, math.pi/2
timestamps, x_vals, y_vals = [], [], []

# Load ROS bag file
bag_path = "moveS.bag"
bag = rosbag.Bag(bag_path)

prev_time = None
data_read = False  # Flag to check if data is actually read

for topic, msg, t in bag.read_messages(topics=["/csc22907/wheels_driver_node/wheels_cmd"]):
    vel_left = msg.vel_left * radius
    vel_right = msg.vel_right * radius

    # Debugging output
    print(f"Time: {t.to_sec()}, Left: {vel_left}, Right: {vel_right}")

    # Compute velocities
    v = (vel_left + vel_right) / 2  # Linear velocity
    omega = (vel_right - vel_left) / WHEEL_BASE  # Angular velocity

    # Convert time from ROS format
    current_time = t.to_sec()
    if prev_time is not None:
        dt = current_time - prev_time
        if dt > 0.1:  # Ignore large gaps
            dt = 0.1
    else:
        dt = 0.01  # Small step for first iteration

    prev_time = current_time
    data_read = True  # Mark that data is actually processed

    # Correct position update using odometry model
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += omega * dt  # Update orientation

    # Store values for plotting
    timestamps.append(current_time)
    x_vals.append(x)
    y_vals.append(y)

bag.close()

# If no data was read, print an error
if not data_read:
    print("No data found in the ROS bag! Check the topic name or bag file.")
    exit()

# Debugging output
print("Final trajectory points:", list(zip(x_vals, y_vals)))

# Plot trajectory
plt.figure(figsize=(8, 8))
plt.plot(x_vals, y_vals, marker="o", linestyle="-", markersize=2)
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Robot Trajectory from Wheel Velocities")
plt.axis("equal")  # Keep equal scaling
plt.grid()
plt.show()
