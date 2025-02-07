#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
import math

# Constants
WHEEL_RADIUS = 0.031  # 3.1 cm
TICKS_PER_REV = 135  # 135 ticks per full wheel rotation
DISTANCE_TARGET = 1.25  # 1.25 meters forward and backward
THROTTLE = 0.4  # Movement speed

class WheelControlNode(DTROS):
    def __init__(self, node_name):
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.vehicle_name = os.environ.get('VEHICLE_NAME', 'duckiebot')
        wheels_topic = f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd"

        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        rospy.Subscriber(f"/{self.vehicle_name}/left_wheel_encoder_node/tick", WheelEncoderStamped, self.left_wheel_callback)
        rospy.Subscriber(f"/{self.vehicle_name}/right_wheel_encoder_node/tick", WheelEncoderStamped, self.right_wheel_callback)

        self.left_ticks_start = None
        self.right_ticks_start = None
        self.left_ticks = 0
        self.right_ticks = 0
        self.distance_traveled = 0
        self.direction = 1  # 1 = Forward, -1 = Backward
        self.completed_reverse = False  # Track whether reverse is completed

        rospy.loginfo("✅ Wheel Control Node Initialized")

    def left_wheel_callback(self, msg):
        rospy.loginfo(f"🔄 Left Encoder: {msg.data}")  # Debug log

        if self.left_ticks_start is None:
            self.left_ticks_start = msg.data  

        self.left_ticks = msg.data - self.left_ticks_start
        self.update_distance()

    def right_wheel_callback(self, msg):
        rospy.loginfo(f"🔄 Right Encoder: {msg.data}")  # Debug log

        if self.right_ticks_start is None:
            self.right_ticks_start = msg.data  

        self.right_ticks = msg.data - self.right_ticks_start
        self.update_distance()

    def update_distance(self):
        self.distance_traveled = max(self.compute_distance(self.left_ticks), self.compute_distance(self.right_ticks))

        rospy.loginfo(f"📏 Distance Traveled: {abs(self.distance_traveled):.3f} meters (Target: {DISTANCE_TARGET}m)")

        # 🚨 Stop exactly at the target distance
        if self.distance_traveled >= DISTANCE_TARGET:
            self.stop()
            rospy.sleep(2)

            if self.direction == 1:  # If we finished moving forward
                self.reverse_direction()
            elif self.direction == -1:  # If we finished moving backward
                rospy.loginfo("✅ Motion Completed Successfully!")
                self.completed_reverse = True
                self.stop()  # Ensure final stop after reversing

    def compute_distance(self, ticks):
        return abs((2 * math.pi * WHEEL_RADIUS * ticks) / TICKS_PER_REV)  # Ensure distance is always positive

    def move(self, vel_left, vel_right):
        message = WheelsCmdStamped()
        message.vel_left = vel_left
        message.vel_right = vel_right
        self._publisher.publish(message)
        rospy.loginfo(f"🚀 Sending Move Command: Left={vel_left}, Right={vel_right}")

    def stop(self):
        rospy.loginfo("🛑 Stopping Duckiebot...")
        self.move(0, 0)
        rospy.sleep(1)

    def reverse_direction(self):
        rospy.loginfo("🔄 Reversing Direction!")
        self.direction = -1  # Switch to reverse
        self.distance_traveled = 0  # Reset traveled distance

        # 🚨 FIX: Reset the encoder reference points correctly
        self.left_ticks_start += self.left_ticks
        self.right_ticks_start += self.right_ticks

        self.left_ticks = 0
        self.right_ticks = 0

        rospy.loginfo("⬅️ Moving Backward...")
        self.move(self.direction * THROTTLE, self.direction * THROTTLE)

    def run(self):
        rospy.loginfo("➡️ Moving Forward...")
        rate = rospy.Rate(10)  

        while not rospy.is_shutdown() and not self.completed_reverse:
            if self.distance_traveled >= DISTANCE_TARGET:  # Stop moving once exact distance is reached
                self.stop()
                break
            self.move(THROTTLE * self.direction, THROTTLE * self.direction)
            rate.sleep()

        self.stop()
        rospy.loginfo("✅ Motion Completed Successfully!")

        # 🚨 Shutdown only AFTER completing both movements
        if self.completed_reverse:
            rospy.signal_shutdown("Finished reversing.")

    def on_shutdown(self):
        rospy.loginfo("🔻 Shutting down the node...")
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)


if __name__ == '__main__':
    node = WheelControlNode(node_name='wheel_control_node')
    node.run()
    rospy.spin()
