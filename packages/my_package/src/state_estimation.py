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
class StateEstimator(DTROS):

    def __init__(self, node_name):
        # Initialize, specify 'node_name' further in 'if __name__ ...'
        super(StateEstimator, self).__init__(node_name=node_name)

        # Initialize variables (once)
        self.node_name = "state_estimation"
        self.veh_name = "maserati4pgts"
        self.number = 0
        self.estimator = False # don't start this node unless 'go' from localization_node

        # Initialize logging services
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        rospy.set_param('/%s/camera_node/res_w' % self.veh_name, 640) # Default is 640px
        rospy.set_param('/%s/camera_node/res_h' % self.veh_name, 480) # Default is 480px

        # List subscribers
        self.sub_camera_image = rospy.Subscriber('/%s/camera_node/image/compressed' % self.veh_name, CompressedImage, self.cbCamera) #from apriltags_postprocessing_node
        self.sub_localization = rospy.Subscriber('/%s/localization_node_test/estimator_trigger' % self.veh_name, BoolStamped, self.cbLocalization)

        #Anti-instagram node sub (corrected, ...)

        # List publishers
        self.pub_localization = rospy.Publisher('/%s/state_estimation/state' %self.veh_name, Int16, queue_size = 1) #if nec, publish only once when goal state is reached, don't publish continuously

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(10)
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
            # Extract necessary image part
            img = self.imageSplitter(img)
            # Count number of blobs (= midline stripes)
            self.number = self.blobCounter(img)
            # Stop when threshold is reached
            return self.number

        else:
            pass


    def imageConverter(self, img):
        # Convert image to cv2
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(img)
            return img
        except CvBridgeError as e:
            rospy.loginfo('Error encountered while converting image')
            return [] #pass


    def colourConverter(self, img):
        # Convert BGR color of image to HSV
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Set boundaries
        lower_yellow = [20, 180, 150]
        upper_yellow = [40, 255, 255]
        mask_yellow = cv2.inRange(imgHSV, lower_yellow, upper_yellow)
        # Output yellow/black image only
        result = cv2.bitwise_and(imgHSV, imgHSV, mask = mask_yellow)
        return result


    def imageSplitter(self, img):
        # Split image
        #imgBOTTOM = np.sum(img[:149,:]==255)
        imgTOP = np.sum(img[590:640,:]==255) #255?
        return imgTOP


    def blobCounter(self, img):
        # WARNING: not robust against noise from other yellow marks (s.a. duckies)
        # Currrently incoming TOPimage
        self.current = np.sum(img)
        # If not black (= yellow)
        if self.current != 0: # For robustness, increase threshold if self.current > some_value:
            # Only count when discontinuity was encountered, and yellow is again True
            if self.go == True:
                self.number = self.number + 1
                # Do not come back, unless fully black image (=line discontinuity) is encountered
                self.go = False
                rospy.loginfo('Reached [-- %s --] stripes, encounting ...' %str(self.number))
                self.pub_localization.publish(self.number)

            else:
                #self.go = False
                pass
        # If black, reset trigger
        else:
            # As soon as image is not fully black anymore, start counting
            self.go = True


# SAFETY & EMERGENCY
    def on_shutdown(self):
        #self.number = 0
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


# KEEP NODE ALIVE
if __name__ == "__main__":
    # Initialize the node with rospy
    node = StateEstimator(node_name="node_name")
    # Keep it spinning to keep the node alive
    rospy.spin()
