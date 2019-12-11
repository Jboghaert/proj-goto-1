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
        self.veh_name = os.environ['VEHICLE_NAME']
        self.number = 0
        self.estimator = False # don't start this node unless 'True' from localization_node
        self.go = False
        self.fsm_mode = "INTERSECTION_CONTROL" #default

        # Initialize logging services
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # Ensure optimal computation, rescale image (only once this node is started, so move this to a callback function)
        rospy.set_param('/%s/camera_node/res_w' % self.veh_name, 640) # Default is 640px
        rospy.set_param('/%s/camera_node/res_h' % self.veh_name, 480) # Default is 480px
        rospy.set_param('/%s/camera_node/framerate' % self.veh_name, 18.) # Minimum is 10-12 Hz (trade-off accuracy-computational power)

        # Correct trim values from terminal for state_estimation (only lane keeping)
        self.se_gain = rospy.get_param('/%s/new_gain' % self.node_name) #default = 0.5
        #self.se_trim = rospy.get_param('/%s/new_trim' % self.node_name)

        # List subscribers
        self.sub_camera_image = rospy.Subscriber('/%s/camera_node/image/compressed' % self.veh_name, CompressedImage, self.cbCamera) #from apriltags_postprocessing_node
        self.sub_localization = rospy.Subscriber('/%s/global_localization/estimator_trigger' % self.veh_name, BoolStamped, self.cbLocalization)
        #self.sub_mode = rospy.Subscriber('/%s/fsm_node/mode' % self.veh_name, FSMState, self.cbMode, queue_size=1)

        #Anti-instagram node sub (corrected, ...)

        # List publishers
        self.pub_localization = rospy.Publisher('/%s/state_estimation/state' %self.veh_name, Int16, queue_size = 1) #if necessary, don't publish continuously (requires import of goal_discrete param)
        self.pub_mask_compressed = rospy.Publisher('/%s/camera_node/mask/compressed' %self.veh_name, CompressedImage, queue_size = 1) #for inspection during testing
        self.pub_crop_compressed = rospy.Publisher('/%s/camera_node/crop/compressed' %self.veh_name, CompressedImage, queue_size = 1) #for inspection during testing

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(25)
        self.bridge = CvBridge()


# CODE GOES HERE

    #def cbMode(self, mode_msg):
        # Get FSM mode
        #self.fsm_mode = mode_msg.state


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
        #if self.fsm_mode != "INTERSECTION_CONTROL" and self.fsm_mode != "INTERSECTION_COORDINATION" and self.fsm_mode != "INTERSECTION_PLANNING":
        #    #only do the following if self.state != intersection something

        if self.estimator == True:
            rospy.loginfo('Intersection navigation finished, going to state estimation')

            # Only once in SE, change kinematic params
            rospy.set_param('/%s/kinematics_node/gain' % self.veh_name, self.se_gain) #desired gain during last mile
            #rospy.set_param('/%s/kinematics_node/trim' % self.veh_name, self.se_trim) #desired trim value

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
        else:
            pass
        #else:
        #    pass


    def imageConverter(self, img):
        # Convert image to cv2
        try:
            cv2_img = self.bridge.compressed_imgmsg_to_cv2(img)
            return cv2_img
        except CvBridgeError as e:
            rospy.loginfo('Error encountered while converting image')
            return [] #pass


    def colourConverter(self, img):
        # Convert BGR color of image to HSV
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Set boundaries (filter out exact HSV values for midline stripes)
        lower_yellow = np.array([30, 80, 180]) #np.uint8
        upper_yellow = np.array([32, 255, 255]) #np.uint8
        mask_yellow = cv2.inRange(imgHSV, lower_yellow, upper_yellow)

        # Output yellow/black image only
        result = cv2.bitwise_and(imgHSV, imgHSV, mask = mask_yellow)
        self.pub_mask_compressed.publish(self.bridge.cv2_to_compressed_imgmsg(result))
        return result


    def imageSplitter(self, img):
        # Publish cropped mask for inspection and tuning of the above interval and framerate
        img_crop_pub = img[390:400,0:400] #lower image part
        self.pub_crop_compressed.publish(self.bridge.cv2_to_compressed_imgmsg(img_crop_pub))

        # Sum hsv values over a few px high image, take out noise from right side (duckies)
        img_crop = np.sum(img[394:400,0:400]==255) #lower image part
        #img_crop = img[394:400,0:400] #lower image part
        return img_crop


    def blobCounter(self, img):
        self.current = np.sum(img)
        rospy.loginfo('Done #3.1')

        # If not black (= yellow)
        if self.current != 0: # For robustness, increase threshold if self.current > some_value:
            # Only count when discontinuity was encountered, and yellow is again True
            rospy.loginfo('Encountered fully black image from mask')
            if self.go == True:
                self.number = self.number + 1
                # Do not come back, unless fully black image (=line discontinuity) is encountered
                self.go = False
                rospy.loginfo('Reached [-- %s --] stripes, encounting ...' % str(self.number))
                self.pub_localization.publish(self.number)
            else:
                #self.go = False
                rospy.loginfo('Huh #1')
                pass

        # If black, reset trigger
        else:
            # As soon as image is not fully black anymore, start counting
            rospy.loginfo('Done #3.2')
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
