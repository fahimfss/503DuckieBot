#!/usr/bin/env python3
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'

import os
import numpy as np
import cv2
from cv_bridge import CvBridge

# Ros libraries
import rospy

# Ros Messages
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

class ImageSubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ImageSubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.sub = rospy.Subscriber("/"+os.environ['VEHICLE_NAME']+"/camera_node/image/compressed", 
        CompressedImage, self.callback,  queue_size = 1)
        self.bridge = CvBridge()
        # dtype, n_channels = self.bridge.encoding_as_cvtype2('8UC3')


    def callback(self, ros_data):
        #### direct conversion to CV2 ####
        # np_arr = np.fromstring(ros_data.data, np.uint8)
        # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image = self.bridge.compressed_imgmsg_to_cv2(ros_data, desired_encoding='rgb8')
        cv2.imwrite("test.jpg", image)
        rospy.loginfo("Image size: %s, Workdir: %s", str(image.shape), os.getcwd())


if __name__ == '__main__':
    # create the node
    node = ImageSubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()