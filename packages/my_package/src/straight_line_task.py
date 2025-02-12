#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import WheelsCmdStamped

# Throttle and direction for each wheel
THROTTLE_LEFT = 0.545  # 50% throttle
DIRECTION_LEFT = 1  # Forward
THROTTLE_RIGHT = 0.5  # 50% throttle
DIRECTION_RIGHT = 1  # Forward

class WheelEncoderReaderNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(WheelEncoderReaderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self._wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"

        # Form the message to run the Duckiebot
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        # Temporary data storage
        self._ticks_left = None
        self._ticks_right = None
        self._initial_ticks_left = None
        self._initial_ticks_right = None

        # Construct subscribers and publishers
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.publisher = rospy.Publisher(self._wheels_topic, WheelsCmdStamped, queue_size=1)

    def callback_left(self, data):
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        if self._initial_ticks_left is None:
            self._initial_ticks_left = data.data
        self._ticks_left = data.data

    def callback_right(self, data):
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        if self._initial_ticks_right is None:
            self._initial_ticks_right = data.data
        self._ticks_right = data.data

    def run(self):
        rate_message = rospy.Rate(20)  # 20 Hz
        rate_cmd = rospy.Rate(1)  # 1 Hz

        # Make sure initial values are not None
        while self._initial_ticks_left is None or self._initial_ticks_right is None:
            rospy.sleep(0.1)

        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)  # Initialize message here

        # While loop for moving forward
        while (self._ticks_left - self._initial_ticks_left) < 750 and (self._ticks_right - self._initial_ticks_right) < 750:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            rate_message.sleep()
            rate_cmd.sleep()

        # Make the robot go reverse
        DIRECTION_LEFT = -1
        DIRECTION_RIGHT = -1
        THROTTLE_LEFT = 0.545  # 50% throttle
        THROTTLE_RIGHT = 0.5  # 50% throttle

        # Form the message to reverse the Duckiebot
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        # While loop for reversing the robot
        while abs(self._ticks_left - self._initial_ticks_left) < 750 and abs(self._ticks_right - self._initial_ticks_right) < 750:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            rate_message.sleep()
            rate_cmd.sleep()

        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        # rotate 90 degrees clockwise
        THROTTLE_RIGHT = 0
        THROTTLE_LEFT = 0.545  # 50% throttle
        DIRECTION_LEFT = 1  # Forward
        DIRECTION_RIGHT = 1  # Forward

        # Form the message to run the Duckiebot
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)

        self._initial_ticks_left = self._ticks_left

        while (self._initial_ticks_left + 35) > self._ticks_left:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            rate_message.sleep()
            rate_cmd.sleep()

        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        # rotate 90 degrees anti-clockwise
        DIRECTION_LEFT = -1

        # Form the message to run the Duckiebot in reverse
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)

        self._initial_ticks_left = abs(self._ticks_left)

        while 35 > abs(self._initial_ticks_left - abs(self._ticks_left)):
            rospy.loginfo(f"Tick difference [LEFT]: {abs(self._ticks_left)}")

            self.publisher.publish(message)

            rate_message.sleep()
            rate_cmd.sleep()

        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        # Shut down the node gracefully
        rospy.loginfo("Shutting down the node, Task completed!")
        rospy.signal_shutdown("Task completed!!")

        

if __name__ == '__main__':
    # create the node
    node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
    # run the timer in node
    node.run()