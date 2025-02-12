#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import ColorRGBA
from led_service import LEDBlinker
#from duckietown_msgs.srv import SetBool, SetBoolResponse


# Throttle and direction for each wheel
THROTTLE_LEFT = 0.33  # 50% throttle
DIRECTION_LEFT = 1  # Forward
THROTTLE_RIGHT = 0.3  # 50% throttle
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

        #initialize the LED_controller
        self.led_controller = LEDBlinker()

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
        self.state_1()
        self.state_2()

            # put shutdown here
            # --># rotate 90 degrees clockwise

    def state_1(self):
        """ Stop the robot and turn LED to a specific color (e.g., RED). """
        rospy.loginfo("State 1: Keeping robot stationary for 5 seconds.")

        # Set LED to red
        self.led_controller.set_led_color("red")

        # Stop wheels
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        rospy.sleep(5)
        rospy.loginfo("State 1 finished.")

    def state_2(self):
        # Tracing “D” path.
        rospy.loginfo("Tracing “D” path.")

        #Set LED to blue
        self.led_controller.set_led_color("blue")
        ###
        # move straight for 1.2m
        ###

        # prepare message to send to wheel subscriber
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)  # Initialize message here

        # update initial values for left and right
        self._initial_ticks_left = self._ticks_left
        self._initial_ticks_right = self._ticks_right

        # While loop for moving forward
        while (self._ticks_left - self._initial_ticks_left) < 720 and (self._ticks_right - self._initial_ticks_right) < 720:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        rospy.sleep(2)

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

        while (self._initial_ticks_left + 99) > self._ticks_left:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        rospy.sleep(2)

        ###
        # move straight for 0.92m
        ###

        THROTTLE_LEFT = 0.33  # 50% throttle
        DIRECTION_LEFT = 1  # Forward
        THROTTLE_RIGHT = 0.30  # 50% throttle
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
        while (self._ticks_left - self._initial_ticks_left) < 500 and (self._ticks_right - self._initial_ticks_right) < 500:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        rospy.sleep(2)

        # # # ###
        # # # # curve right 
        # # # ###

        # update the parameters
        THROTTLE_RIGHT = 0.2
        THROTTLE_LEFT = 0.47  # 50% throttle
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
        while (self._ticks_left - self._initial_ticks_left) < 240 or (self._ticks_right - self._initial_ticks_right) < 120:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        rospy.sleep(2)


        # ###
        # # move straight for 0.61m
        # ###

        THROTTLE_LEFT = 0.33  # 50% throttle
        DIRECTION_LEFT = 1  # Forward
        THROTTLE_RIGHT = 0.3  # 50% throttle
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
        while (self._ticks_left - self._initial_ticks_left) < 380 and (self._ticks_right - self._initial_ticks_right) < 380:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        rospy.sleep(2)


        # # ###
        # # # curve right 
        # # ###

        # update the parameters
        THROTTLE_RIGHT = 0.2
        THROTTLE_LEFT = 0.48  # 50% throttle
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
        while (self._ticks_left - self._initial_ticks_left) < 240 or (self._ticks_right - self._initial_ticks_right) < 120:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        rospy.sleep(2)



        ###
        # move straight for 0.92m
        ###

        THROTTLE_LEFT = 0.33  # 50% throttle
        DIRECTION_LEFT = 1  # Forward
        THROTTLE_RIGHT = 0.3  # 50% throttle
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
        while (self._ticks_left - self._initial_ticks_left) < 500 and (self._ticks_right - self._initial_ticks_right) < 500:
            rospy.loginfo(f"Tick difference [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        rospy.sleep(2)



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

        while (self._initial_ticks_left + 99) > self._ticks_left:
            rospy.loginfo(f"Tick differendocker cp ed4670dbe7dd:/code/catkin_ws/src/dt-gui-tools/move.bag /home/sandhya/my-ros-projectce [LEFT]: {self._ticks_left - self._initial_ticks_left}")
            rospy.loginfo(f"Tick difference [RIGHT]: {self._ticks_right - self._initial_ticks_right}")
            self.publisher.publish(message)

            self.rate_message.sleep()
            # self.rate_cmd.sleep()

        # make the robot to stop by setting both wheel speed to 0
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop_cmd)

        # After the final rotation is complete
        rospy.sleep(5)

        self.led_controller.set_led_color("red")
        self.publisher.publish(stop_cmd)

  


        


if __name__ == '__main__':
    # create the node
    trajectory_node = DTrajectory(node_name='d_trajectory')

    # run the timer in node
    trajectory_node.run()