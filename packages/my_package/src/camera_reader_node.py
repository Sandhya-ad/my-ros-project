#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._processed_image_topic = f"/{self._vehicle_name}/camera_processed/image/compressed"

        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        # self._window = "camera-reader"
        #cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
         # Subscribe to the camera topic
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

        # Publisher for the processed image
        self.pub = rospy.Publisher(self._processed_image_topic, CompressedImage, queue_size=1)

    def callback(self, msg):
        try:
            # Convert JPEG bytes to OpenCV image
            image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Get image size
            height, width, _ = image.shape
            print(f"Received image size: {width}x{height}")

            # Convert to grayscale
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Convert grayscale image back to 3-channel (for annotation display)
            gray_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)

            # Annotate image with text
            text1 = f"Duck {self._vehicle_name} says,"
            text2 = f"'Cheese! Capturing {width}x{height} - quack-tastic!'"
            font = cv2.FONT_HERSHEY_SIMPLEX 
            font_scale =  min(width, height) / 700
            font_thickness = int(font_scale * 2)
            text_color = (0, 255, 0)  # Green text
            
            text_x = 10
            text_y = height - 40  # Place near bottom
            
            # Put text on image
            cv2.putText(gray_image, text1, (text_x, text_y), font, font_scale, text_color, font_thickness, cv2.LINE_AA)
            cv2.putText(gray_image, text2, (text_x, height - 20), font, font_scale, text_color, font_thickness, cv2.LINE_AA)

            processed_msg = self._bridge.cv2_to_compressed_imgmsg(gray_image)
            self.pub.publish(processed_msg)
            # Display the annotated grayscale image
            # cv2.imshow(self._window, gray_image)
            # cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()