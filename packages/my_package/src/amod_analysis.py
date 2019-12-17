#!/usr/bin/env python

# NOTE:
# This is not part of the code for GOTO-1, but rather an additional project for time allocation analysis of indefinite_navigation,
# and subsequent visualization of the results. This was at the time of the DEMO unfinished. If all relevant/main nodes are linked
# to a specific state, we could simply subscribe to the FSM alone and time the different states (first cb function in this node).
# Key is to select the states that are relevant and are as MECE as possible (no overlap, else ambiguity in the results).

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
        super(AmodAnalyzer, self).__init__(node_name=node_name)

        # Initialize variables (once)
        self.node_name = "state_estimation"
        self.veh_name = os.environ['VEHICLE_NAME']

        # Initialize duration
        self.FSM_elapsed = 0
        self.Loc_elapsed = 0
        self.Vel_elapsed = 0

        # Initialize logging services
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # List subscribers (selection of indefinite_navigation nodes)
        self.sub_localization = rospy.Subscriber('/%s/global_localization/estimator_trigger' % self.veh_name, BoolStamped, self.cbLoc)
        self.sub_state_estimator = rospy.Subscriber('/%s/global_localization/estimator_trigger' % self.veh_name, BoolStamped, self.cbSE)
        self.sub_wheel_actuation = rospy.Subscriber('something' % self.veh_name, BoolStamped, self.cbVel)

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(20)


# CODE GOES HERE
    # Take cumulative duration of different FSMstates
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

    # Take cumulative duration of active localization_node (rather, publish to FSM and subscribe only to FSM)
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
