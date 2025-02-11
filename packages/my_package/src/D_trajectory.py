#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import ColorRGBA
#from duckietown_msgs.srv import SetBool, SetBoolResponse


# Throttle and direction for each wheel
THROTTLE_LEFT = 0.545  # 50% throttle
DIRECTION_LEFT = 1  # Forward
THROTTLE_RIGHT = 0.5  # 50% throttle
DIRECTION_RIGHT = 1  # Forward


class DTrajectory(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(DTrajectory, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

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

        # Construct subscribers and publishers related to wheels
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
        self.rate_message = rospy.Rate(20)  # 20 Hz
        # self.rate_cmd = rospy.Rate(20)  # 1 Hz

        # Make sure initial values are not None
        while self._initial_ticks_left is None or self._initial_ticks_right is None:
            rospy.sleep(0.1)

        self.wheel_radius = rospy.get_param(f"/{self._vehicle_name}/kinematics_node/radius")
        rospy.loginfo_once(f"Wheel radius : {self.wheel_radius}")

        self.wheel_base = 10  # cm

        # while not rospy.is_shutdown():
        # self.state_1()
        self.state_2()

            # put shutdown here
            # --># rotate 90 degrees clockwise

    def state_1(self):
        # keep the robot stationary for 5 seconds.
        rospy.loginfo("Keeping robot stationary for 5 seconds.")

        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        # set the LED lights to green color for state 1
        # TODO

        # Sleep for 5 seconds to keep the robot stationary
        rospy.sleep(5)

        # keep the robot stationary for 5 seconds.
        rospy.loginfo("State 1 finished.")

    def state_2(self):
        # Tracing “D” path.
        rospy.loginfo("Tracing “D” path.")

        # set the LED lights to blue color for state 2
        # TODO

        ###
        # move straight for 1.2m
        ###

        # prepare message to send to wheel subscriber
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)  # Initialize message here

        # update initial values for left and right
        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        # While loop for moving forward
        while (self._ticks_left - self._initial_ticks_left) < 750 and (self._ticks_right - self._initial_ticks_right) < 750:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        ###
        # rotate 90 degrees clockwise
        ###

        # update the parameters
        THROTTLE_RIGHT = 0
        THROTTLE_LEFT = 0.5  # 50% throttle
        DIRECTION_LEFT = 1  # Forward
        DIRECTION_RIGHT = 1  # Forward

        # update the wheel speed parameters
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        # update the message
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)

        self._initial_ticks_left = self._ticks_left

        while (self._initial_ticks_left + 107) > self._ticks_left:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        ###
        # move straight for 0.92m
        ###

        THROTTLE_LEFT = 0.545  # 50% throttle
        DIRECTION_LEFT = 1  # Forward
        THROTTLE_RIGHT = 0.5  # 50% throttle
        DIRECTION_RIGHT = 1  # Forward

        # update the wheel speed parameters
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        # prepare message to send to wheel subscriber
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)  # Initialize message here

        # update initial values for left and right
        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        # While loop for moving forward
        while (self._ticks_left - self._initial_ticks_left) < 550 and (self._ticks_right - self._initial_ticks_right) < 550:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        # # ###
        # # # curve right 
        # # ###

        # update the parameters
        THROTTLE_RIGHT = 0.25
        THROTTLE_LEFT = 0.545  # 50% throttle
        DIRECTION_LEFT = 1  # Forward
        DIRECTION_RIGHT = 1  # Forward

        # update the wheel speed parameters
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        # prepare message to send to wheel subscriber
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)  # Initialize message here

        # update initial values for left and right
        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        # While loop for moving forward
        while (self._ticks_left - self._initial_ticks_left) < 208 or (self._ticks_right - self._initial_ticks_right) < 104:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)


        ###
        # move straight for 0.61m
        ###

        THROTTLE_LEFT = 0.545  # 50% throttle
        DIRECTION_LEFT = 1  # Forward
        THROTTLE_RIGHT = 0.5  # 50% throttle
        DIRECTION_RIGHT = 1  # Forward

        # update the wheel speed parameters
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        # prepare message to send to wheel subscriber
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)  # Initialize message here

        # update initial values for left and right
        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        # While loop for moving forward
        while (self._ticks_left - self._initial_ticks_left) < 370 and (self._ticks_right - self._initial_ticks_right) < 370:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)


        # # ###
        # # # curve right 
        # # ###

        # update the parameters
        THROTTLE_RIGHT = 0.25
        THROTTLE_LEFT = 0.545  # 50% throttle
        DIRECTION_LEFT = 1  # Forward
        DIRECTION_RIGHT = 1  # Forward

        # update the wheel speed parameters
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        # prepare message to send to wheel subscriber
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)  # Initialize message here

        # update initial values for left and right
        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        # While loop for moving forward
        while (self._ticks_left - self._initial_ticks_left) < 208 or (self._ticks_right - self._initial_ticks_right) < 104:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)



        ###
        # move straight for 0.92m
        ###

        THROTTLE_LEFT = 0.545  # 50% throttle
        DIRECTION_LEFT = 1  # Forward
        THROTTLE_RIGHT = 0.5  # 50% throttle
        DIRECTION_RIGHT = 1  # Forward

        # update the wheel speed parameters
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        # prepare message to send to wheel subscriber
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)  # Initialize message here

        # update initial values for left and right
        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        # While loop for moving forward
        while (self._ticks_left - self._initial_ticks_left) < 550 and (self._ticks_right - self._initial_ticks_right) < 550:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)



        ###
        # rotate 90 degrees clockwise
        ###

        # update the parameters
        THROTTLE_RIGHT = 0
        THROTTLE_LEFT = 0.5  # 50% throttle
        DIRECTION_LEFT = 1  # Forward
        DIRECTION_RIGHT = 1  # Forward

        # update the wheel speed parameters
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT

        # update the message
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)

        self._initial_ticks_left = self._ticks_left

        while (self._initial_ticks_left + 107) > self._ticks_left:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        


if __name__ == '__main__':
    # create the node
    trajectory_node = DTrajectory(node_name='d_trajectory')

    # run the timer in node
    trajectory_node.run()
