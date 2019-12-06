# Guideline for testmodule 1.0 of project goto-1 (AMOD 2019)
Author: J. Boghaert
Tutor: M. Hosner, G. Zardini
Group: GOTO-1

## Implementation
The scripts within proj-goto-1 are written for the 2019 Duckietown class at ETH Zürich. It should be implemented in the existing framework of **indefinite navigation**. For the requirements, see section 'Important'.

## Guideline
Carefully follow the steps below to implement the proj-goto-1 solution onto your duckiebot.
- [] Read the README.md file
- [] Read the cmd.txt file

## Important
This code assumes the following:
- DT map as hardcoded in path_planning_class
- no other AT's present as the ones hardcoded in path_planning_class
- no varying lighting conditions
- no external factors (s.a. obstacles)
- a well functioning indefinite_navigation demo version with ROS graph as attached

## Content
### my_package
This package contains 2 nodes and an external class for path planning using a predefined map of DT.

#### localization_node
This node **localizes** the duckiebot and uses an external path planning class to generate the **shortest path** to get from the localized point to a given destination point. It is the main code of proj-goto-1 and generates the desired turn commands at each intersection. The driving input of this code are the intersection AT's.

**Note:**
The code itself explains in- and output arguments, as well as additional information on the exact approach and reasoning behind it.

#### state_estimation_node
This node executes the **last mile** problem of proj-goto-1 by converting the input distance (from a certain AT) to passing a desired number of midline stripes.