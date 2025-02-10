#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from duckietown_msgs.msg import WheelsCmdStamped


# throttle and direction for each wheel
THROTTLE_LEFT = 0.5        # 50% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.5       # 50% throttle
DIRECTION_RIGHT = 1       # forward


class WheelEncoderReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelEncoderReaderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
      
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self._wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"

        # form the message to run the duckiebot
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT
      
        # temporary data storage
        self._ticks_left = None
        self._ticks_right = None
        self._initial_ticks_left = None
        self._initial_ticks_right = None
      
        # construct subscriber and publishers
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.publisher = rospy.Publisher(self._wheels_topic, WheelCmdStamped, queue_size=1)

    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        # store data value
        if self._initial_ticks_left == None:
          self._initial_ticks_left = data.data
        self._ticks_left = data.data

    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        # store data value
        if self._initial_ticks_right == None:
          self._initial_ticks_right = data.data
        self._ticks_right = data.data

    def run(self):
        # publish received tick messages every 0.05 second (20 Hz)
        rate_message = rospy.Rate(20)
        # publish 10 messages every second (10 Hz)
        run_cmd = rospy.Rate(0.1)
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=sel            self._publisher.publish(message)f._vel_right)
        while (self._ticks_left - self._initial_ticks_left) < 805 and (self._ticks_right - self._initial_ticks_right) < 805:
            if self._ticks_right is not None and self._ticks_left is not None and self._initial_ticks_left is not None and self.__initial_ticks_right is not None:
                # start printing values when received from both encoders
                msg = f"Wheel encoder ticks [LEFT, RIGHT]: {self._ticks_left}, {self._ticks_right}"
                rospy.loginfo(msg)
                self.publisher.publish(message)
              
            rate_message.sleep()
            rate_cmd.sleep()
          
        stop_cmd = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
    # run the timer in node
    node.run()
    # keep spinning
    rospy.spin()
