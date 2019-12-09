#!/usr/bin/env python
# Search for TODO, or correct to see issues to be resolved

# ----------------------------------------------------------------------------------------------------------------------------------------------------- #


# IMPORT
import numpy as np
import os
import rospy
import yaml
import time
import cv2

from duckietown import DTROS
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetection, TurnIDandType
from duckietown_msgs.msg import BoolStamped, FSMState
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import WheelsCmdStamped

from cv_bridge import CvBridge


# INITIATE DTROS CLASS (incl sub/pub)
class AmodAnalyzer(DTROS):

    def __init__(self, node_name):
        # Initialize, specify 'node_name' further in 'if __name__ ...'
        super(AmodAnalyzer, self).__init__(node_name=node_name)

        # Initialize variables (once)
        self.node_name = "state_estimation"
        self.veh_name = "maserati4pgts"

        # Initialize logging services
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # Ensure optimal computation, rescale image (only once this node is started, so move this to a callback function)
        rospy.set_param('/%s/camera_node/res_w' % self.veh_name, 640) # Default is 640px
        rospy.set_param('/%s/camera_node/res_h' % self.veh_name, 480) # Default is 480px
        rospy.set_param('/%s/camera_node/framerate' % self.veh_name, 20.) # Minimum is 10-12 Hz (trade-off accuracy-computational power)

        # List subscribers
        self.sub_camera_image = rospy.Subscriber('/%s/camera_node/image/compressed' % self.veh_name, CompressedImage, self.cbCamera) #from apriltags_postprocessing_node
        self.sub_localization = rospy.Subscriber('/%s/localization_node_test/estimator_trigger' % self.veh_name, BoolStamped, self.cbLocalization)

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(20)
        self.bridge = CvBridge()


# CODE GOES HERE

    def cbLocalization(self, msg):
        # Keep this true, independent from new message
        self.estimator = True

        # WARNING: only update image here (else all camera feed for all nodes is of low quality)
        # Ensure optimal computation, rescale image
        #rospy.set_param('/%s/camera_node/res_w' % self.veh_name, 160) # Default is 640px
        #rospy.set_param('/%s/camera_node/res_h' % self.veh_name, 120) # Default is 480px
        #rospy.set_param('/%s/camera_node/framerate' % self.veh_name, 15.) # Minimum is 10-12 Hz (trade-off accuracy-computational power)

        # Create timer to update params of camera_node
        #rospy.Timer(rospy.Duration.from_sec(2.0), self.updateParams)


    def cbCamera(self, img):
        if self.estimator == True:

            rospy.loginfo('Preparing image')
            # Convert to OpenCV image in HSV
            img = self.colourConverter(self.imageConverter(img))
            rospy.loginfo('Done #1')
            # Extract necessary image part, sum HSV values
            sum = self.imageSplitter(img)
            rospy.loginfo('Done #2')
            # Count number of blobs (= midline stripes)
            self.blobCounter(sum)
            rospy.loginfo('Done #3')
            # Stop when threshold is reached
            #return self.number

        else:
            pass


# SAFETY & EMERGENCY
    def on_shutdown(self):
        #self.number = 0
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


# KEEP NODE ALIVE
if __name__ == "__main__":
    # Initialize the node with rospy
    node = AmodAnalyzer(node_name="node_name")
    # Keep it spinning to keep the node alive
    rospy.spin()
