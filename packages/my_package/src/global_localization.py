#!/usr/bin/env python
# Search for TODO, or correct to see issues to be resolved


# TITLE: TRIGGER FUNCTION TO START LOCALIZATION? PATH PLANNING AND PUBLISH WHEEL CMD

# DESCRIPTION:
# This script takes input from apriltags_postprocessing_node and checks if a localizable AT (that is included in our predefined map) is reached
# If so, it produces a usable input to the path_planning module for further processing and therefore acts as a switch, if not, AT_detection and lane_following continue
# The path_planning module returns the shortest path and executable wheel commands that are then published to unicorn_intersection_node
# If the final AT is reached, a state estimation function (listening to the camera_node topic) is started (issue: actions wrt. both subscribers are coupled)

# TODO:
# Include os.environ to automatically update DB name - make all parameters dynamic
# Fix state state_estimation
# Include analysis of AmOD
# Include dynamic trim/gain tuner - include tune_file.txt
# Include that when pressing S in joystick, the SE stops as well (as does the AT input)
# Check for lag between camera and actual
# Only do SE when not in self.state = intersection something

# ----------------------------------------------------------------------------------------------------------------------------------------------------- #


# IMPORT
import numpy as np
import os
import rospy
import yaml
import time
import math

from duckietown import DTROS
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetection, TurnIDandType
from duckietown_msgs.msg import BoolStamped, FSMState
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped

from path_planning_class import PathPlanner


# INITIATE DTROS CLASS (incl sub/pub)
class LocalizationNode(DTROS):

    def __init__(self, node_name):
        # Initialize, specify 'node_name' further in 'if __name__ ...'
        super(LocalizationNode, self).__init__(node_name=node_name)

        # Initialize variables
        self.node_name = "global_localization"
        self.veh_name = os.environ['VEHICLE_NAME']
        # Related to path planning
        self.AT = False #start localization or not
        self.new_AT = True #start execution or not
        self.plan = True #start planning or not
        self.full_path = []
        self.path = []
        self.cmd = []
        self.turn_type = -1
        # Related to state estimation
        self.estimation = False #start state estimation or not
        self.state = False #destination reached or not
        self.stripe_length = (5. + 2.5) #stripe length in cm

        # Initialize logging services
        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # Get arrival point (from roslaunch/docker cmd setting the parameter 'goal_input' in terminal)
        self.goal = rospy.get_param('/%s/goal_input' % self.node_name) # of type final AT id [int32]
        self.goal_distance = rospy.get_param('/%s/goal_distance' % self.node_name) # of type [distance after 2nd to last AT (actually take stopline)] in cm
        self.goal_discrete = int(self.goal_distance / self.stripe_length) + 3 # discretize length in cm to number of stripes, compensate view/delays with 3 additional stripes for actual position

        # Import from external class PathPlanner as pp
        self.pp = PathPlanner()
        self.tags = self.pp.tags
        self.graph = self.pp.graph


        # Adjustment for intersection navigation (passed from terminal) - could also be done by remapping in launch file !!
        self.ff_left = rospy.get_param('/%s/inter_nav_ff_left' % self.node_name) #default = 0.4
        self.ff_right = rospy.get_param('/%s/inter_nav_ff_right' % self.node_name) #default = -0.6
        self.time_l_turn = rospy.get_param('/%s/inter_nav_time_left_turn' % self.node_name) #default = 3.2
        self.time_r_turn = rospy.get_param('/%s/inter_nav_time_right_turn' % self.node_name) #default = 1.5

        rospy.set_param('/%s/unicorn_intersection_node/ff_left' % self.veh_name, self.ff_left) #desired values during intersection navigation
        rospy.set_param('/%s/unicorn_intersection_node/ff_right' % self.veh_name, self.ff_right)
        rospy.set_param('/%s/unicorn_intersection_node/time_left_turn' % self.veh_name, self.time_l_turn)
        rospy.set_param('/%s/unicorn_intersection_node/time_right_turn' % self.veh_name, self.time_r_turn)


        # List subscribers
        self.sub_AT_detection = rospy.Subscriber('/%s/apriltags_postprocessing_node/apriltags_out' %self.veh_name, AprilTagsWithInfos, self.callback) #from apriltags_postprocessing_node
        self.sub_mode = rospy.Subscriber('/%s/fsm_node/mode' % self.veh_name, FSMState, self.cbMode, queue_size=1)
        self.sub_state_estimator = rospy.Subscriber('/%s/state_estimation/state' % self.veh_name, Int16, self.cbState)

        # List publishers
        self.pub_direction_cmd = rospy.Publisher('/%s/random_april_tag_turns_node/turn_id_and_type' % self.veh_name, TurnIDandType, queue_size = 1) # to unicorn_intersection_node
        self.pub_wheels_cmd = rospy.Publisher("/%s/wheels_driver_node/wheels_cmd" % self.veh_name, WheelsCmdStamped, queue_size = 1) # for emergency stop, else use onShutdown
        #self.pub_override_joystick = rospy.Publisher('/%s/joy_mapper_node/joystick_override' % self.veh_name, BoolStamped, queue_size = 1) # necessary?
        #self.pub_turn_type = rospy.Publisher("/%s/turn_type" % self.veh_name, Int16, queue_size=1) #unnecessary
        self.pub_state_estimator = rospy.Publisher('/%s/global_localization/estimator_trigger' % self.veh_name, BoolStamped, queue_size = 1)

        # Conclude
        rospy.loginfo("[%s] Initialized." % (self.node_name))
        self.rate = rospy.Rate(15)


# CODE GOES HERE
    def cbState(self, msg):
        # Upon incoming msg, stop main callback by setting self.state = True
        self.state = True
        rospy.Rate(40)
        rospy.loginfo('We reached now %s stripes, out of %s' %(msg.data, self.goal_discrete))

        # Permanently subscribe to avoid AT cb during state_estimator from beginning, not only when final goal was reached
        if msg.data >= self.goal_discrete:
            #stop DB
            self.publishStop()
            rospy.loginfo('You have reached your destination')
            #stop StateEstimator
            self.estimation = False
            self.publishTrigger(self.estimation)
            #say goodbye
            rospy.loginfo('Thank you for driving with %s in Duckietown, enjoy your stay!' % self.veh_name)
            self.onShutdown()
        else:
            pass


    def cbMode(self, mode_msg):
        # Get FSM mode
        self.fsm_mode = mode_msg.state
        # Filter out INTERSECTION_CONTROL modes
        if(self.fsm_mode != mode_msg.INTERSECTION_CONTROL):
            self.turn_type = -1
            #self.pub_turn_type.publish(self.turn_type)


    def callback(self, msg):
        # Check if state_estimation node is active
        if self.state == False: #while?

            if self.fsm_mode == "INTERSECTION_CONTROL" or self.fsm_mode == "INTERSECTION_COORDINATION" or self.fsm_mode == "INTERSECTION_PLANNING":
                # loop through list of april tags

                rospy.loginfo('Incoming AprilTagsWithInfos msg, starting callback ...')
                # filter out the nearest apriltag
                dis_min = 999
                idx_min = -1
                for idx, taginfo in enumerate(msg.infos):
                    rospy.loginfo('Proceed: #0')
                    if(taginfo.tag_type == taginfo.SIGN):
                        tag_det = (msg.detections)[idx]
                        pos = tag_det.pose.pose.position
                        distance = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                        if distance < dis_min:
                            dis_min = distance
                            idx_min = idx
                        else:
                            rospy.loginfo('What to do here: #1')

                    else:
                        rospy.loginfo('What to do here: #2')
                        pass

                # continue iff
                if idx_min != -1:
                    taginfo = (msg.infos)[idx_min]

                    # upon first AT - localize and generate path
                    if self.plan == True:
                        rospy.loginfo('Starting detection if AT is readible')
                        trigger, starting_point = self.detection(msg)

                        if trigger == True: # detected AT is in list, use as starting point
                            self.plan = False # don't come back here
                            rospy.loginfo('Starting localization module ...')
                            # Define path and cmd list after localization of AT if AT went through the above filter first
                            # Save generated path and turn_cmd globally
                            path, self.cmd = self.pathPlanning(trigger, starting_point)

                            # Take out the goal_input AT, which was only used to get all turn_cmds
                            path.pop() # take out last AT entry (only necessary to calculate turn_cmds)
                            self.full_path = path # fixed
                            self.path = path # updated throughout script

                            if len(self.path) > 1: #can be 1, since last AT is omitted from self.path
                                turn_cmd = self.pathProcessor(self.path, self.cmd)
                                self.publishCmd(turn_cmd)

                            elif len(self.path) == 1: # Publish final turn command
                                turn_cmd = self.pathProcessor(self.path, self.cmd)
                                self.publishCmd(turn_cmd)
                                rospy.loginfo('Continue to StateEstimator after first AT')

                                time.sleep(5)
                                self.estimation = True
                                self.publishTrigger(self.estimation)
                                self.state = True


                            else: # Stop immediately
                                rospy.loginfo('Check this out: unexpected/unidentified behaviour #1')
                                self.publishStop()

                    # upon all next AT's - execute generated path
                    else:
                        rospy.loginfo('Starting detection if next AT is next in path sequence')
                        trigger = self.executeFilter(msg)

                        if trigger == True: # detected AT is (next) in cmd list
                            rospy.loginfo('Starting execution module ...')

                            if len(self.path) > 1:
                                turn_cmd = self.pathProcessor(self.path, self.cmd)
                                self.publishCmd(turn_cmd)
                            elif len(self.path) == 1: # Publish final turn command
                                turn_cmd = self.pathProcessor(self.path, self.cmd)
                                self.publishCmd(turn_cmd)
                                rospy.loginfo('Continue to StateEstimator')

                                time.sleep(5)
                                self.estimation = True
                                self.publishTrigger(self.estimation)
                                self.state = True


                            else: # Stop immediately
                                rospy.loginfo('Check this out: unexpected/unidentified behaviour #2')
                                self.publishStop()

                        else:
                            rospy.loginfo('It is not')
                            #rospy.loginfo('What to do here: #3')
                            pass
                else:
                    rospy.loginfo('What to do here: #4')
                    pass
        else:
            rospy.loginfo('Encountered an AT while in state estimation, ignoring AT input')
            pass


    def detection(self, msg): # Process incoming msg from ~apriltags_out of type AprilTagsWithInfos
        for item in msg.detections:
            if item.id in self.tags: #check if closest AT is localizable
                self.AT = True
                # Return usable input
                starting_point = item.id
                rospy.loginfo('Readible AT [%s] successfully detected, proceeding to correct module of localization/execution ...' %str(starting_point))
                return self.AT, starting_point
            else:
                self.AT = False
                outlying_point = item.id
                rospy.loginfo('Incorrectly oriented AT [%s] detected, passing AT == False message (ignore) ...' %str(item.id))
                return self.AT, outlying_point


    def executeFilter(self, msg):
        #self.previous = list(set(self.full_path).symmetric_difference(set(self.path))) #sequence of AT's popped out
        #self.remaining = list(set(self.full_path).symmetric_difference(set(self.previous))) #update continuously = self.path

        for item in msg.detections:
            if item.id in self.path: #check if AT has already been encountered
                rospy.loginfo('self.new_AT = True with id [%s]' %item.id)
                self.new_AT = True
                return self.new_AT
            elif item.id == self.goal:
                rospy.loginfo('self.new_AT = False with id [%s]' %item.id)
                rospy.loginfo('We reached the final goal input node: stop now!')
                self.publishStop()
                self.new_AT = False
                return self.new_AT
            else:
                # AT is either unidentified (not in list), or in list (but not in path anymore)
                rospy.loginfo('self.new_AT = False with id [%s]' %item.id)
                self.new_AT = False
                return self.new_AT


    def pathPlanning(self, AT, starting_point): # Calculate shortest path from current AT in time
        if AT == True:
            rospy.loginfo('Starting path planning')
            # Define start and end point (dynamic)
            start = starting_point
            goal = self.goal
            # Run path planning from imported class (defined in initializer)
            path, cmd = self.pp.dijkstra(start, goal)
            rospy.loginfo('Confirmation: Path successfully generated ! With starting point the detected AT id %s' %str(start))
            return path, cmd
        else:
            rospy.loginfo('No usable AT detected, ignore message ... ')


    def pathProcessor(self, path, cmd):
        # Take current node and command
        # NOTE: since path and cmd are 'self.' defined, it is unnecessary to pass them as variables
        cmd_comb = [path[0], cmd[0]]
        # Set up outgoing message type
        new_cmd = TurnIDandType()
        new_cmd.tag_id = cmd_comb[0]

        if cmd_comb[1] == 0:
            new_cmd.turn_type = 0
            rospy.loginfo("Turn left")
        elif cmd_comb[1] == 1:
            new_cmd.turn_type = 1
            rospy.loginfo("Go straight")
        elif cmd_comb[1] == 2:
            new_cmd.turn_type = 2
            rospy.loginfo("Turn right")
        return new_cmd


    def publishCmd(self, new_cmd):
        self.pub_direction_cmd.publish(new_cmd)
        #self.pub_turn_type.publish(new_cmd.turn_type)
        rospy.loginfo("Published turn_cmd and turn_type")
        # Once published, keep self.plan == False
        self.plan = False
        self.AT = False #unnecessary since self.plan already is False
        # Update sequence (note: self.path updates itself!)
        self.path.pop(0)
        self.cmd.pop(0)
        rospy.loginfo('Updated remaining path and cmd')


    def publishTrigger(self, bool):
        triggerCmd = BoolStamped()
        triggerCmd.data = bool
        self.pub_state_estimator.publish(triggerCmd)


# REACHING FINAL POINT
    def publishStop(self):
        # Produce wheel stopping cmd vel(0,0)
        stop_cmd = WheelsCmdStamped()
        stop_cmd.vel_left = 0.0
        stop_cmd.vel_right = 0.0
        self.pub_wheels_cmd.publish(stop_cmd)
        rospy.loginfo("Published stop_cmd")


# SAFETY & EMERGENCY
    def on_shutdown(self):
        self.publishStop()
        rospy.loginfo("[%s] Shutting down." % (self.node_name)) #correct?


# KEEP NODE ALIVE
if __name__ == "__main__":
    # Initialize the node with rospy
    node = LocalizationNode(node_name="node_name")
    #rospy.on_shutdown(LocalizationNode.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
