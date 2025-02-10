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

        rospy.loginfo("Wheel Control Node Initialized")

    def left_wheel_callback(self, msg):
        if self.left_ticks_start is None:
            self.left_ticks_start = msg.data  

        self.left_ticks = msg.data - self.left_ticks_start
        self.update_distance()

    def right_wheel_callback(self, msg):
        if self.right_ticks_start is None:
            self.right_ticks_start = msg.data  

        self.right_ticks = msg.data - self.right_ticks_start
        self.update_distance()

    def update_distance(self):
        self.distance_traveled = max(self.compute_distance(self.left_ticks), self.compute_distance(self.right_ticks))

        rospy.loginfo(f"📏 Distance Traveled: {abs(self.distance_traveled):.3f} meters (Target: {DISTANCE_TARGET}m)")

        if self.distance_traveled >= DISTANCE_TARGET:
            self.stop()
            rospy.sleep(2)  # Pause before reversing

            if self.direction == 1:  # If moving forward, switch to reverse
                self.move_reverse()
            else:  # If moving backward, stop completely
                rospy.loginfo("✅ Motion Completed Successfully!")
                self.completed_reverse = True
                rospy.signal_shutdown("Finished reversing.")

    def compute_distance(self, ticks):
        return abs((2 * math.pi * WHEEL_RADIUS * ticks) / TICKS_PER_REV)

    def reset_encoders(self):
        """ Resets encoder values to ensure correct distance calculation. """
        self.left_ticks_start = None
        self.right_ticks_start = None
        self.left_ticks = 0
        self.right_ticks = 0

    def move(self, vel_left, vel_right):
        """ Sends movement commands to the wheels. """
        message = WheelsCmdStamped()
        message.vel_left = vel_left
        message.vel_right = vel_right
        self._publisher.publish(message)

    def stop(self):
        """ Stops the robot and ensures it doesn't move again. """
        self.move(0, 0)
        rospy.loginfo("🛑 Duckiebot Stopped.")
        rospy.sleep(2)

    def move_forward(self):
        """ Moves the robot forward for 1.25 meters. """
        rospy.loginfo("➡️ Moving Forward...")
        self.direction = 1
        self.distance_traveled = 0
        self.reset_encoders()
        self.move(THROTTLE, THROTTLE)

    def move_reverse(self):
        """ Moves the robot backward for 1.25 meters. """
        rospy.loginfo("⬅️ Moving Backward...")
        self.direction = -1
        self.distance_traveled = 0
        self.reset_encoders()
        self.move(-THROTTLE, -THROTTLE)

    def run(self):
        """ Main execution loop for moving forward and then reversing. """
        self.move_forward()
        rate = rospy.Rate(12)  # 10 Hz loop

        while not rospy.is_shutdown() and not self.completed_reverse:
            rate.sleep()

        if self.completed_reverse:
            rospy.signal_shutdown("Finished reversing.")

    def on_shutdown(self):
        """ Ensures the robot stops safely before shutting down. """
        self.stop()
        rospy.loginfo("🔻 Node shutting down...")

if __name__ == '__main__':
    node = WheelControlNode(node_name='wheel_control_node')
    node.run()
    rospy.spin()
