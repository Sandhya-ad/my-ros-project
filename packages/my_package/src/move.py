#!/usr/bin/env python3

import os
import rospy
import math
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

# Constants
WHEEL_RADIUS = 0.031  # 3.1 cm
TICKS_PER_REV = 135  # Encoder ticks per full wheel rotation
WHEEL_BASE = 0.1  # Distance between wheels
DISTANCE_TARGET = 1.25  # Distance to move forward and backward
TARGET_ANGLE = math.pi / 2  # 90 degrees in radians
THROTTLE = 0.4  # Speed for movement
ROTATION_SPEED = 0.5  # Speed for rotation

class MoveNode(DTROS):
    def __init__(self, node_name):
        super(MoveNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.vehicle_name = os.environ.get('VEHICLE_NAME', 'duckiebot')
        wheels_topic = f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd"

        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        rospy.Subscriber(f"/{self.vehicle_name}/left_wheel_encoder_node/tick", WheelEncoderStamped, self.callback)
        rospy.Subscriber(f"/{self.vehicle_name}/right_wheel_encoder_node/tick", WheelEncoderStamped, self.callback)

        self.left_ticks_start = None
        self.right_ticks_start = None
        self.left_ticks = 0
        self.right_ticks = 0
        self.distance_traveled = 0
        self.angle_rotated = 0
        self.completed_straight = False
        self.completed_reverse = False
        self.completed_rotation = False

        rospy.on_shutdown(self.on_shutdown)  # Ensure safe stopping on shutdown

        rospy.loginfo("MoveNode Initialized")

    def callback(self, msg):
        """ Handles encoder tick updates. """
        if "left" in msg._connection_header['topic']:  # Left wheel encoder
            if self.left_ticks_start is None:
                self.left_ticks_start = msg.data  
            self.left_ticks = msg.data - self.left_ticks_start
        else:  # Right wheel encoder
            if self.right_ticks_start is None:
                self.right_ticks_start = msg.data  
            self.right_ticks = msg.data - self.right_ticks_start
                
        if not self.completed_straight:
            self.compute_distance_traveled()
        elif not self.completed_reverse:
            self.compute_distance_reversed()
        elif not self.completed_rotation:
            self.update_rotation()

    def compute_distance_traveled(self):
        """ Computes and tracks the distance moved forward in meters. """
        self.distance_traveled = max(self.compute_distance(self.left_ticks), self.compute_distance(self.right_ticks))
        rospy.loginfo(f"📏 Distance Traveled: {self.distance_traveled:.3f} meters (Target: {DISTANCE_TARGET}m)")

        if self.distance_traveled >= DISTANCE_TARGET:
            self.stop()
            rospy.sleep(2)  # Pause before switching direction
            self.completed_straight = True
            self.reset_encoders()
            rospy.loginfo("⬅️ Starting Reverse Motion...")
            self.drive_straight(reverse=True)

    def compute_distance_reversed(self):
        """ Computes and tracks the distance moved in reverse. """
        self.distance_traveled = max(self.compute_distance(self.left_ticks), self.compute_distance(self.right_ticks))
        rospy.loginfo(f"⬅️ Distance Reversed: {self.distance_traveled:.3f} meters (Target: {DISTANCE_TARGET}m)")

        if self.distance_traveled >= DISTANCE_TARGET:
            self.stop()
            rospy.sleep(2)  # Pause before rotating
            self.completed_reverse = True
            self.reset_encoders()
            rospy.loginfo("➡️ Starting Rotation...")
            self.rotate_clockwise()

    def update_rotation(self):
        """ Calculates the current rotation angle and checks if the target angle is reached. """
        left_distance = self.compute_distance(self.left_ticks)
        right_distance = self.compute_distance(self.right_ticks)

        self.angle_rotated = abs((left_distance - right_distance) / WHEEL_BASE)
        rospy.loginfo(f"🔄 Angle Rotated: {self.angle_rotated:.3f} radians (Target: {TARGET_ANGLE:.2f} radians)")

        if self.angle_rotated >= TARGET_ANGLE:
            self.stop()
            rospy.sleep(2)  # Pause before stopping fully
            self.completed_rotation = True
            rospy.loginfo("✅ Rotation Task Completed Successfully!")
            rospy.signal_shutdown("Finished all tasks.")

    def compute_distance(self, ticks):
        """ Converts encoder ticks into linear distance. """
        return abs((2 * math.pi * WHEEL_RADIUS * ticks) / TICKS_PER_REV)

    def reset_encoders(self):
        """ Resets encoder start values to ensure correct tracking. """
        self.left_ticks_start = None
        self.right_ticks_start = None
        self.left_ticks = 0
        self.right_ticks = 0

    def drive_straight(self, reverse=False):
        """ Moves the Duckiebot forward or backward. """
        rospy.loginfo(f"🚗 {'Reversing' if reverse else 'Moving Forward'}...")
        self.reset_encoders()  # Ensure fresh encoder readings before moving
        self.distance_traveled = 0  # Reset distance tracking
        speed = -THROTTLE if reverse else THROTTLE
        self.move(speed, speed)

    def rotate_clockwise(self):
        """ Rotates the Duckiebot 90 degrees clockwise. """
        rospy.loginfo("➡️ Rotating 90 degrees Clockwise...")
        self.angle_rotated = 0  # Only track rotation with angle
        self.reset_encoders()
        self.move(ROTATION_SPEED, -ROTATION_SPEED)  # Left wheel forward, Right wheel backward

    def move(self, vel_left, vel_right):
        """ Sends movement commands to the wheels. """
        message = WheelsCmdStamped()
        message.vel_left = vel_left
        message.vel_right = vel_right
        self._publisher.publish(message)

    def stop(self):
        """ Stops the Duckiebot. """
        self.move(0, 0)
        rospy.loginfo("🛑 Stopped.")
        rospy.sleep(1)

    def on_shutdown(self):
        """ Ensures the robot stops when shutting down. """
        rospy.loginfo("🔻 Shutting down, stopping Duckiebot...")
        self.stop()

    def run(self):
        rospy.sleep(2)  # Allow node initialization

        # Task 1: Move forward 1.25m
        self.drive_straight()
        while not rospy.is_shutdown() and not self.completed_straight:
            rospy.sleep(0.1)  # Loop delay for checking

        # Task 2: Move backward 1.25m
        while not rospy.is_shutdown() and not self.completed_reverse:
            rospy.sleep(0.1)

        # Task 3: Rotate 90 degrees clockwise
        while not rospy.is_shutdown() and not self.completed_rotation:
            rospy.sleep(0.1)

        self.stop()
        rospy.loginfo("✅ Rotation Task Completed Successfully!")

if __name__ == '__main__':
    node = MoveNode(node_name='move_node')
    node.run()
    rospy.spin()
