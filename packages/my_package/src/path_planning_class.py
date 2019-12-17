#!/usr/bin/env python

# TITLE: SHORTEST PATH PLANNER GIVEN INPUT AT, PREDEFINED MAP AND COST FUNCTION (~ distance, #turns)

# DESCRIPTION:
# This script is called by and takes input from global_localization in GOTO-1 and calculates the shortest path (in our predefined map)
# based on Dijkstra's Algorithm. It obeys basic traffic rules, s.a. lane direction and no U-turns (see user scenario description of map)
# It also takes into account the distances AND the number of turns between nodes (AT's) as weights/scaled costs for the SSP problem.
# Output is a sequence of AT's (nodes) the DB has to follow in order to get to the desired end point via the shortest path.

# ----------------------------------------------------------------------------------------------------------------------------------------------------- #


# IMPORT
import numpy as np
import os
import rospy


# MAP SPECIFIC (CHANGE THIS IF MAP CHANGES)
class PathPlanner:

    def __init__(self):
        # List all AT id's (int32[] type) that are considered for localization purposes in the predefined map
        # Add '0' to make the index of i refer to the i-th value from the python array
        self.tags = [0, 11, 61, 9, 199, 15, 8, 231, 66, 63, 59]
        self.number_of_tags = len(self.tags)

        iD = self.tags

        # Define weights/costs between AT's to base SPP on - this is the predefined DT map (manual input)
        self.graph = {
            iD[1]: {iD[4]: 6.7, iD[9]: 8},
            iD[2]: {iD[5]: 2.7, iD[9]: 7.3},
            iD[3]: {iD[4]: 6.8, iD[5]: 2.9},

            iD[4]: {iD[1]: 2.9, iD[8]: 3,   iD[10]: 7.5},
            iD[5]: {iD[2]: 9.1, iD[8]: 2.9, iD[10]: 6.8},
            iD[6]: {iD[1]: 3.7, iD[2]: 8.4, iD[10]: 6.7},
            iD[7]: {iD[1]: 3,   iD[2]: 8.3, iD[8]: 3.7},

            iD[8]: {iD[3]: 6.4, iD[7]: 7.5},
            iD[9]: {iD[7]: 8.4, iD[6]: 3.7},
            iD[10]: {iD[3]: 6.5, iD[6]: 2.9}}

        # Define allowable turn commands between AT's to base SPP execution on
        self.graph_direction = {
            iD[1]: {iD[4]: 2, iD[9]: 0},
            iD[2]: {iD[5]: 0, iD[9]: 1},
            iD[3]: {iD[4]: 1, iD[5]: 2},

            iD[4]: {iD[1]: 2, iD[8]: 1, iD[10]: 0},
            iD[5]: {iD[2]: 0, iD[8]: 2, iD[10]: 1},
            iD[6]: {iD[1]: 0, iD[2]: 1, iD[10]: 2},
            iD[7]: {iD[1]: 1, iD[2]: 2, iD[8]: 0},

            iD[8]: {iD[3]: 2, iD[7]: 0},
            iD[9]: {iD[7]: 1, iD[6]: 0},
            iD[10]: {iD[3]: 1, iD[6]: 2}}


# MAIN CODE (DO NOT CHANGE)
    def dijkstra(self, start, goal):
        # Set up
        shortest_distance = {}
        predecessor = {}
        infinity = 9999999
        path = []
        cmd = []
        rospy.loginfo('Starting path planning, using Dijkstra ...')

        # Dijkstra algorithm
        for node in self.graph:
            shortest_distance[node] = infinity
        shortest_distance[start] = 0

        while self.graph:
            minNode = None
            for node in self.graph:
                if minNode is None:
                    minNode = node
                elif shortest_distance[node] < shortest_distance[minNode]:
                    minNode = node

            for childNode, weight in self.graph[minNode].items():
                if weight + shortest_distance[minNode] < shortest_distance[childNode]:
                    shortest_distance[childNode] = weight + shortest_distance[minNode]
                    predecessor[childNode] = minNode
            self.graph.pop(minNode) # do not recalculate for same AT - might have implications on other ATs as well!!

        currentNode = goal
        while currentNode != start:
            try:
                path.insert(0, currentNode)
                cmd.insert(0, self.graph_direction[predecessor[currentNode]][currentNode])
                currentNode = predecessor[currentNode]
            except KeyError:
                rospy.loginfo('Path not reachable')
                break

        path.insert(0, start)
        if shortest_distance[goal] != infinity:
            rospy.loginfo('Shortest distance is ' + str(shortest_distance[goal]))
            rospy.loginfo('And the path is ' + str(path))
            rospy.loginfo('And the sequence of turn cmds is ' + str(cmd))

        return path, cmd
