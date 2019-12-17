#!/usr/bin/env python

# TITLE: AUXILIARY NODE FOR GOTO-1: STATE FEEDBACK, RESOLVING LAST MILE PROBLEM

# DESCRIPTION:
# This script is called by the global_localization node from GOTO-1, and counts the number of midline stripes once the intersection navigation is done.
# It acts as a state feedback mechanism (state = distance to final arrival point B). The global_localization node publishes a stop cmd when B is reached.

# ----------------------------------------------------------------------------------------------------------------------------------------------------- #


# IMPORT
import numpy as np
import os
import rospy
import yaml
import time
import cv2

from duckietown import DTROS
from duckietown_msgs.msg import BoolStamped, FSMState
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage, Image

from cv_bridge import CvBridge


# INITIATE DTROS CLASS (incl sub/pub)
class StateEstimator(DTROS):

    def __init__(self, node_name):
        super(StateEstimator, self).__init__(node_name=node_name)

        # Initialize variables (once)
        self.node_name = "state_estimation"
        self.veh_name = os.environ['VEHICLE_NAME']
        self.number = 0
        self.estimator = False #Don't start this node unless 'True' from global_localization
        self.go = False #Start counting again
        self.fsm_mode = "INTERSECTION_CONTROL" #Default, else false callback if cbLocalization happens before cbMode

        # Initialize logging services
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # List subscribers
        self.sub_camera_image = rospy.Subscriber('/%s/camera_node/image/compressed' % self.veh_name, CompressedImage, self.cbCamera) #from apriltags_postprocessing_node
        self.sub_localization = rospy.Subscriber('/%s/global_localization/estimator_trigger' % self.veh_name, BoolStamped, self.cbLocalization)
        self.sub_mode = rospy.Subscriber('/%s/fsm_node/mode' % self.veh_name, FSMState, self.cbMode)

        # List publishers
        self.pub_localization = rospy.Publisher('/%s/state_estimation/state' %self.veh_name, Int16, queue_size = 1)
        self.pub_mask_compressed = rospy.Publisher('/%s/camera_node/mask/compressed' %self.veh_name, CompressedImage, queue_size = 1) #for inspection during testing
        self.pub_crop_compressed = rospy.Publisher('/%s/camera_node/crop/compressed' %self.veh_name, CompressedImage, queue_size = 1) #for inspection during testing


        # DEMO SPECIFIC PARAMETER TUNING
        # Ensure optimal computation (e.g. lower/higher image resolution)
        rospy.set_param('/%s/camera_node/res_w' % self.veh_name, 640) # Default is 640px
        rospy.set_param('/%s/camera_node/res_h' % self.veh_name, 480) # Default is 480px
        rospy.set_param('/%s/camera_node/framerate' % self.veh_name, 18.) # Minimum is 10-12 Hz (trade-off accuracy-computational power)
        # Correct linear velocity for last mile state (only lane keeping)
        self.se_v_bar = rospy.get_param('/%s/new_v_bar' % self.node_name) # Default is 0.23


        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        rospy.Rate(30)
        self.bridge = CvBridge()


# CODE GOES HERE

    def cbMode(self, mode_msg):
        # Get FSM mode
        self.fsm_mode = mode_msg.state


    def cbLocalization(self, msg):
        # Set trigger (bool) to start cbCamera or not
        self.estimator = msg.data
        rospy.loginfo('Trigger = %s' %msg.data)


    def cbCamera(self, img):
        # Only start estimator once intersection navigation is over (inaccurate, as intersection navigation is a feedfwd cmd)
        if self.fsm_mode != "INTERSECTION_CONTROL" and self.fsm_mode != "INTERSECTION_COORDINATION" and self.fsm_mode != "INTERSECTION_PLANNING":

            # Only start if in last mile
            if self.estimator == True:
                # Only once in SE, limit linear velocity during last mile
                rospy.set_param('/%s/lane_controller_node/v_bar' % self.veh_name, self.se_v_bar)
                rospy.loginfo('Preparing image')

                # Convert to OpenCV image in HSV
                img = self.colourConverter(self.imageConverter(img))
                # Extract necessary image part, sum HSV values
                sum = self.imageSplitter(img)
                # Count number of blobs (= midline stripes)
                self.blobCounter(sum)
                rospy.loginfo('Done #3')

            else:
                pass
        else:
            pass


    def imageConverter(self, img):
        # Convert image to cv2
        try:
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(img)
            return cv2_img
        except CvBridgeError as e:
            rospy.loginfo('Error encountered while converting image')
            return []


    def colourConverter(self, img):
        # Convert BGR to HSV
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Crop image to reduce computation, take out noise from duckies on right side
        imgHSV = img[400:410,0:400]

        # Set boundaries (filter out exact HSV values for midline stripes)
        lower_yellow = np.array([29, 80, 180]) #np.uint8
        upper_yellow = np.array([33, 255, 255]) #np.uint8
        mask_yellow = cv2.inRange(imgHSV, lower_yellow, upper_yellow)

        # Output yellow/black image only
        result = cv2.bitwise_and(imgHSV, imgHSV, mask = mask_yellow)
        return result


    def imageSplitter(self, img):
        # For inspection only
        img_crop_pub = img[8:10,:] #Further reduce image size
        self.pub_crop_compressed.publish(self.bridge.cv2_to_compressed_imgmsg(img_crop_pub))

        # Sum values
        img_crop_sum = np.sum(img_crop_pub)
        return img_crop_sum


    def blobCounter(self, sum):
        self.current = sum

        # If not black (= yellow)
        if self.current != 0: # For robustness, increase threshold "if self.current > some_value":

            # Only count when discontinuity was encountered, and yellow is again True
            rospy.loginfo('Encountered fully black image from mask')
            if self.go == True:
                self.number = self.number + 1
                # Do not come back, unless fully black image (=line discontinuity) is encountered
                self.go = False
                # Give feedback
                rospy.loginfo('Reached [-- %s --] stripes, encounting ...' % str(self.number))
                self.pub_localization.publish(self.number)
            else:
                rospy.loginfo('Check this out - unidentified behaviour')
                pass

        # If black, reset trigger
        else:
            # As soon as image is not fully black anymore, start counting
            self.go = True


# SAFETY & EMERGENCY
    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


# KEEP NODE ALIVE
if __name__ == "__main__":
    # Initialize the node with rospy
    node = StateEstimator(node_name="node_name")
    # Keep it spinning to keep the node alive
    rospy.spin()
