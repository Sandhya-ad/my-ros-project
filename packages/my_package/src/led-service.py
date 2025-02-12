#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from try_led import LEDBlinker  # Import LED control
import time

class LEDTestController(DTROS):  # Single DTROS node
    def __init__(self):
        super(LEDTestController, self).__init__(node_name="led_test_controller", node_type=NodeType.GENERIC)

        self.led_controller = LEDBlinker()  # Use LED control

    def run(self):
        rospy.loginfo("Starting LED test")

        # Cycle through colors every 3 seconds for 30 seconds
        colors = ["red", "green", "blue", "yellow", "purple"]
        start_time = time.time()
        index = 0

        while time.time() - start_time < 30:  # Run for 30 seconds
            self.led_controller.set_led_color(colors[index % len(colors)])
            rospy.loginfo(f"Changed LED to {colors[index % len(colors)]}")
            index += 1
            time.sleep(3)  # Wait 3 seconds before changing the color

        rospy.loginfo("LED test completed.")

if __name__ == "__main__":
    node = LEDTestController()
    node.run()
