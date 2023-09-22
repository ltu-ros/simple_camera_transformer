#!/usr/bin/env python3
import rospy
import cv2
import pdb
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from simple_camera_transformer.cfg import TransformerConfig
import numpy as np

class image_transformer:
    def __init__(self):
        # Initization of the node, name_sub
        rospy.init_node('cam_transformer', anonymous=True)

        # Get rosparams
        if rospy.has_param('~sub_topic'):
            sub_topic=rospy.get_param('~sub_topic')      
        else:
            print("Missing sub_topic parameter")
            exit(-1)

        if rospy.has_param('~pub_topic'):
            pub_topic=rospy.get_param('~pub_topic')      
        else:
            print("Missing pub_topic parameter")
            exit(-1)
            
        # Initialize the CvBridge class
        self.bridge = CvBridge()

        # Setup config server
        self.config=None
        self.cfg_srv = Server(TransformerConfig, self.configCallback)

        # Setup callback function
        self.image_sub = rospy.Subscriber(sub_topic, Image, self.image_callback)

        # Setup publisher
        self.pub = rospy.Publisher(pub_topic, Image, queue_size=10)

        rospy.spin()

    def configCallback(self, cfg, level):
        self.config=cfg
        return cfg

    def applyTransform(self, frame):
        if self.config is None:
            return frame

        # Resize
        frame=cv2.resize(frame, (0,0), fx=self.config.resize, fy=self.config.resize, interpolation=cv2.INTER_AREA)

        # Color correction (brightness & contrast)
        if self.config.enable_color_correct:
            frame = (frame*self.config.cc_alpha + self.config.cc_beta).astype(np.uint8)

        # Sharpen
        if self.config.enable_sharpen:
            out=cv2.GaussianBlur(frame, (0,0), self.config.sharp_kernel*2+1)
            frame = cv2.addWeighted( frame, 1.0+self.config.sharp_weight, out, -1.0*self.config.sharp_weight, 0).astype(np.uint8)

        return frame

    def image_callback(self, img_msg):
        # Convert the ROS Image message to a CV2 Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))    
            return    
        frame = self.applyTransform(cv_image)
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame,encoding='bgr8'))
            
if __name__ == '__main__':
    IT=image_transformer()
 
