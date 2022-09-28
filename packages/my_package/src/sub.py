#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.bridge_ = CvBridge()
        self.sub = rospy.Subscriber('/duckiebot02/camera_node/image/compressed', CompressedImage, self.callback)


    def callback(self, data):
        # rospy.loginfo("I heard %s", data.data)
        self.latest_image_ = data
        
        try:
            # cv_image = self.bridge_.imgmsg_to_cv2(self.latest_image_)
            # np_arr = np.fromstring(data.data, np.uint8)
            # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
            # print(np_arr.size)
            cv_image = self.bridge_.compressed_imgmsg_to_cv2(self.latest_image_)
            print(cv_image.shape[:2])
        except CvBridgeError as e:
            print(e)
            return

        print('recive image ~')

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()