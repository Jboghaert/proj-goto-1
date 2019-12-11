<div figure-id="fig:header">
     <img src="media/header.png" style='width: 20em'/>
</div>


# Global Localization GOTO-1
Author: J. Boghaert

Tutors: M. Hosner, G. Zardini

Add Description & objective


# Content & pipeline structure
Within the `packages/my_package/src` directory, 2 ROS nodes and an external class for path planning can be found.

<div figure-id="fig:pipeline_vis">
     <img src="media/pipeline_vis.png" style='width: 20em'/>
</div>


#### 1. localization_node
This node **localizes** the duckiebot and uses an external path planning class to generate the **shortest path** to get from the localized point to a given destination point. It is the main code of proj-goto-1 and generates the desired turn commands at each intersection. The driving input of this code are the intersection AT's.

**Note:**
The code itself explains in- and output arguments, as well as additional, more detailed information on the exact approach and reasoning behind the code.

#### 2. path_planning_class
This class is imported by `localization_node` and calculates the **shortest path** given an input and output node within the predefined DT map.

#### 3. state_estimation_node
This node executes the **last mile** problem of proj-goto-1 by converting the input distance (from a certain AT) to passing a desired number of midline stripes.





## Implementation prerequisites
The scripts within the GOTO-1 project are written for the 2019 Duckietown (AMOD) class at ETH Zürich. The entire project is based on a ROS-template providing a boilerplate repository for developing ROS-based software in Duckietown, to be found [here](https://github.com/duckietown/template-ros).

Running the project should be implemented in the existing framework of `indefinite_navigation`, more info to be found [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/demo_indefinite_navigation.html). This framework allows us to comply with the Duckietown traffic rules, lane following and the necessary task prioritization of incoming commands. The implementation of the GOTO-1 project requires some changes to be made within the indefinite navigation framework, which are outlined in the next sections.

### Setting up the framework
Include ros graph default

### Implementing GOTO-1
Include ros graph with altered structure



## Guideline
Carefully follow the steps below to implement the proj-goto-1 solution onto your duckiebot.
- [ ] Read the README.md file
- [ ] Scan through the scripts and change the name of the DB if necessary
- [ ] Read the cmd.txt file
- [ ] Add any desired or necessary extensions to your operating system (s.a. dts shell, docker, ...)
- [ ] Execute the cmd.txt file and change all DB dependent parameters if necessary (s.a. IP address and name)

## Important
This code assumes the following:
- DT map as hardcoded in path_planning_class
- no other AT's present as the ones hardcoded in path_planning_class
- no varying lighting conditions
- no external factors (s.a. obstacles)
- a well functioning indefinite_navigation demo version with ROS graph as attached


## Troubleshooting
As the existing framework of `indefinite_navigation` is not stable, the scripts for GOTO-1 can overrule the gain and trim values with new values passed through the command terminal.