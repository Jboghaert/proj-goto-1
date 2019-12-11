#!/usr/bin/env python
# Search for TODO, or correct to see issues to be resolved

# ----------------------------------------------------------------------------------------------------------------------------------------------------- #


# IMPORT
import numpy as np
import os
import rospy
import time

import matplotlib as plt

from duckietown import DTROS
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetection, TurnIDandType
from duckietown_msgs.msg import BoolStamped, FSMState
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import WheelsCmdStamped


# INITIATE DTROS CLASS (incl sub/pub)
class AmodAnalyzer(DTROS):

    def __init__(self, node_name):
        # Initialize, specify 'node_name' further in 'if __name__ ...'
        super(AmodAnalyzer, self).__init__(node_name=node_name)

        # Initialize variables (once)
        self.node_name = "state_estimation"
        self.veh_name = os.environ['VEHICLE_NAME']

        self.FSM_elapsed = 0
        self.Loc_elapsed = 0
        self.Vel_elapsed = 0

        # Initialize logging services
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # Ensure optimal computation, rescale image (only once this node is started, so move this to a callback function)
        rospy.set_param('/%s/camera_node/res_w' % self.veh_name, 640) # Default is 640px
        rospy.set_param('/%s/camera_node/res_h' % self.veh_name, 480) # Default is 480px
        rospy.set_param('/%s/camera_node/framerate' % self.veh_name, 20.) # Minimum is 10-12 Hz (trade-off accuracy-computational power)

        # List subscribers
        self.sub_localization = rospy.Subscriber('/%s/localization_node_test/estimator_trigger' % self.veh_name, BoolStamped, self.cbLoc)
        self.sub_state_estimator = rospy.Subscriber('/%s/localization_node_test/estimator_trigger' % self.veh_name, BoolStamped, self.cbSE)
        self.sub_wheel_actuation = rospy.Subscriber('something' % self.veh_name, BoolStamped, self.cbVel)

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(20)
        self.bridge = CvBridge()


# CODE GOES HERE

    def cbFSM(self, FSMmsg):
        while FSMmsg == active:
            while FSMmsg.something == something:
                    # Define locally
                    FSM_start = time.time #in seconds
                    FSM_elapsed = time.time - start
                    # Define globally - continuously update
                    self.FSM1_elapsed = self.FSM1_elapsed + FSM_elapsed

            while FSMmsg.something == something_else:
                    # Define locally
                    FSM_start = time.time
                    FSM_elapsed = time.time - start
                    # Define globally - continuously update
                    self.FSM2_elapsed = self.FSM1_elapsed + FSM_elapsed

            while FSMmsg.something == something_else:
                    # Define locally
                    FSM_start = time.time
                    FSM_elapsed = time.time - start
                    # Define globally - continuously update
                    self.FSM3_elapsed = self.FSM1_elapsed + FSM_elapsed
        else:
            pass


    def cbLoc(self, Locmsg):
        while Locmsg == active:
            # Define locally
            FSM_start = time.time
            FSM_elapsed = time.time - start
            # Define globally - continuously update
            self.Loc_elapsed = self.Loc_elapsed + FSM_elapsed
        else:
            pass


    def cbVel(self, Velmsg):
        while Velmsg == active:
            # Define locally
            Vel_start = time.time
            Vel_elapsed = time.time - start
            # Define globally - continuously update
            self.Vel_elapsed = self.Vel_elapsed + FSM_elapsed
        else:
            pass


    def plotAmod(self):
        # Set up state space (FSM1,2,3, Loc and Vel)
        x_val = [1, 2, 3, 4, 5]
        y_val = [self.FSM1_elapsed, self.FSM2_elapsed, self.FSM3_elapsed, self.Loc_elapsed, self.Vel_elapsed]
        # Compare separate states (MECE) and wheels active/non-active
        y_label = ['Intersection Active', 'State Estimator Active', 'Indefinite Navigation Active', 'Localization Active', 'Wheels Active']

        # Plot values upon shutdown
        plt.bar(x_val, y_val, tick_label = y_label, width = 0.5, color = ['blue', 'green'])
        plt.xlabel('Duckiebot states')
        plt.ylabel('Duration (s)')
        plt.title('Time spent by %s per state' %self.veh_name)


# SAFETY & EMERGENCY
    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


# KEEP NODE ALIVE
if __name__ == "__main__":
    # Initialize the node with rospy
    node = AmodAnalyzer(node_name="node_name")
    # Keep it spinning to keep the node alive
    rospy.spin()
