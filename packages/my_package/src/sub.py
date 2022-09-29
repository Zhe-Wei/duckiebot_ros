#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

os.environ['DISPLAY'] = ':0'
cv2.VideoCapture(0,1,"")

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.bridge_ = CvBridge()
        self.sub = rospy.Subscriber('/duckiebot02/camera_node/image/compressed', CompressedImage, self.callback)


    def callback(self, data):
        self.latest_image_ = data
        
        try:
            cv_image = self.bridge_.compressed_imgmsg_to_cv2(self.latest_image_)
            print(cv_image.shape)
            cv2.imshow("Image", cv_image)
            cv2.waitKey(1) 
        except CvBridgeError as e:
            print(e)
            return

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()