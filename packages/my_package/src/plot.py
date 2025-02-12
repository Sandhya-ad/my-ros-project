# #!/usr/bin/env python3

# import rosbag
# import numpy as np
# import matplotlib.pyplot as plt
# from nav_msgs.msg import Odometry

# # Initialize lists for plotting
# x_vals, y_vals = [], []
# bag_file = "move.bag"  # Change this to your actual bag file path
# topic_name = "/csc22907/deadreckoning_node/odom"  # C

# def compute_trajectory():
#     bag = rosbag.Bag('move.bag')

#     print("Reading position data from bag file...")

#     for topic, msg, t in bag.read_messages(topics=['/csc22907/deadreckoning_node/odom']):
#         x = msg.pose.pose.position.x  # Read x directly from position
#         y = msg.pose.pose.position.y  # Read y directly from position

#         x_vals.append(x)
#         y_vals.append(y)

#         print(f"Time: {t.to_sec():.2f}, X: {x:.3f}, Y: {y:.3f}")

#     bag.close()

# def plot_trajectory():
#     compute_trajectory()

#     plt.figure(figsize=(8, 6))
#     plt.plot(x_vals, y_vals, marker='o', linestyle='-', markersize=3, label="Duckiebot Trajectory")
#     plt.xlabel("X Position (m)")
#     plt.ylabel("Y Position (m)")
#     plt.title("Duckiebot Trajectory from Odometry")
#     plt.legend()
#     plt.grid()
#     plt.show()

# if __name__ == '__main__':
#     plot_trajectory()
import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Constants
WHEEL_BASE = 0.2  # Adjust this based on your robot's real dimensions

# Initialize position and heading
x, y, theta = 0.0, 0.0, 0.0
timestamps, x_vals, y_vals = [], [], []

# Load ROS bag file
bag_path = "move_D.bag"
bag = rosbag.Bag(bag_path)

prev_time = None
data_read = False  # Flag to check if data is actually read

for topic, msg, t in bag.read_messages(topics=["/csc22907/wheels_driver_node/wheels_cmd"]):
    vel_left = msg.vel_left
    vel_right = msg.vel_right

    # Debugging output
    print(f"Time: {t.to_sec()}, Left: {vel_left}, Right: {vel_right}")

    # Compute velocities
    v = (vel_left + vel_right) / 2  
    omega = (vel_right - vel_left) / WHEEL_BASE

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

    # Flip theta if needed (negate omega)
    theta += omega * dt  # If movement is flipped, change to theta -= omega * dt

    # Update position (swap X and Y if necessary)
    y += v * np.cos(theta) * dt
    x += v * np.sin(theta) * dt

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

# Flip Y-axis if needed

plt.grid()
plt.show()
