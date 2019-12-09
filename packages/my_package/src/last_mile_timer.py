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
class LastMileTimer(DTROS):

    def __init__(self, node_name):
        # Initialize, specify 'node_name' further in 'if __name__ ...'
        super(LastMileTimer, self).__init__(node_name=node_name)

        # Initialize variables (once)
        self.node_name = "last_mile_timer"
        self.veh_name = "maserati4pgts"
        self.counter = False # don't start this node unless 'True' from localization_node

        # Initialize logging services
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # Get parameters
        rospy.get_param('/%s/kinematics_node/gain' % self.veh_name)
        rospy.get_param('/%s/kinematics_node/trim' % self.veh_name)
        self.goal_distance = rospy.get_param('/%s/goal_distance' % self.node_name)

        # Calculate stop time
        self.

        # List subscribers
        self.sub_localization = rospy.Subscriber('/%s/localization_node_test/estimator_trigger' % self.veh_name, BoolStamped, self.cbLocalization)

        # List publishers
        self.pub_wheels = rospy.Publisher('/%s/state_estimation/state' %self.veh_name, Int16, queue_size = 1) #if nec, publish only once when goal state is reached, don't publish continuously

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(20)
        self.bridge = CvBridge()


# CODE GOES HERE

    def cbLocalization(self, msg):
        self.counter = True


    def pubWheelCmd(self, img):
        if self.estimator == True:

            rospy.loginfo('Calculating stop time')
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
    node = LastMileTimer(node_name="node_name")
    # Keep it spinning to keep the node alive
    rospy.spin()
