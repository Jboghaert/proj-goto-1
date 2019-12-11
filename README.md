<div figure-id="fig:header">
     <img src="media/header.png" style='width: 20em'/>
</div>


# Global Localization GOTO-1
Author: J. Boghaert

Tutors: M. Hosner, G. Zardini

Add Description & objective
The goal of this project is to navigate a single Duckiebot within a predefined Duckietown lay-out from any starting point A to a randomly generated arrival point B.


## Content & pipeline structure
Within the `packages/my_package/src` directory, all nodes and external classes for the GOTO-1 project can be found. The figure below shows the overall pipeline of the project. It can be seen that the altered `indefinite_navigation` module is running all the time. In addition, the joystick controller is used to trigger and overrule the GOTO-1 modules whenever necessary.

<div figure-id="fig:pipeline_vis">
     <img src="media/pipeline_vis.png" style='width: 20em'/>
</div>

**Legenda:** Unmarked inputs are given or self-determined, yellow blocks are running in the existing framework of `indefinite_navigation`, and orange/grey-marked items represent the custom blocks developed for GOTO-1.

#### 1. global_localization_node
This node **localizes** the duckiebot and uses an external path planning class to generate the **shortest path** to get from the localized point to a given destination point. It is the main code of GOTO-1 and generates the desired turn commands at each intersection, as well as the stop command upon arrival. The driving input of this code are the intersection AT's, serving as nodes for the `path_planning_class` outlined next.

**Note:**
The code itself explains in- and output arguments, as well as additional, more detailed information on the exact approach and reasoning behind the code.

#### 2. path_planning_class
This class is imported by `localization_node` and calculates the **shortest path** given an input and output node within the predefined DT map. The predefined DT map is hardcoded in this class.

#### 3. state_estimation_node
This node executes the **last mile** problem of proj-goto-1 by converting the input distance (from a certain AT) to passing a desired number of midline stripes and visually counting these until the desired position is reached.


## Implementation prerequisites
The scripts within the GOTO-1 project are written for the 2019 Duckietown (AMOD) class at ETH Zürich. The entire project is based on a ROS-template providing a boilerplate repository for developing ROS-based software in Duckietown, to be found [here](https://github.com/duckietown/template-ros).

Running the project should be implemented in the existing framework of `indefinite_navigation`, more info to be found [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/demo_indefinite_navigation.html). This framework allows us to comply with the Duckietown traffic rules, lane following and the necessary task prioritization of incoming commands. The implementation of the GOTO-1 project requires some changes to be made within the indefinite navigation framework, which are outlined in the next sections.

### Setting up the framework
Before building anything, make sure to be connected to your Duckiebot, and retrieve its IP address through
```
$ ping DUCKIEBOT_NAME.local
```
Then, make sure to pull the latest docker images for `dt-core`, `dt-car-interface` and `dt-duckiebot-interface` through
```
$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-car-interface:daffy
$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-duckiebot-interface:daffy
$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-core:daffy
```

Include ros graph default


### Implementing GOTO-1

Now build the image:
```
$ dts devel watchtower stop -H DUCKIEBOT_NAME.local
$ chmod +x ./packages/my_package/src/localization_node.py
$ chmod +x ./packages/my_package/src/state_estimation.py
$ dts devel build -f --arch arm32v7 -H DUCKIEBOT_NAME.local
```
Then run the GOTO-1 module, and access its root to pass the desired input commands:
```
$ docker -H DUCKIEBOT_NAME.local run -it --name proj-goto-1 --privileged -v /data:/data -e ROS_MASTER_URI=http://DUCKIEBOT_IP:11311/ --rm --net host duckietown/IMAGE_NAME:IMAGE_TAG /bin/bash
$ roslaunch my_package proj_goto_1.launch goal_input:="199" goal_distance:="40"
```

<html>
<head>
<style>
* {box-sizing: border-box;}

.column {float: left; width: 33.33%; padding: 5px;}

.row::after {content: ""; clear: both; display: table;}
</style>
</head>
<body>

<div class="row">
  <div class="column">
    <img src="media/global_localization.png" alt="ROS1" style='width:50%'>
  </div>
  <div class="column">
    <img src="media/state_estimation.png" alt="ROS2" style="width:50%">
  </div>
</div>

</body>
</html>



### Stopping procedure:
When stopping the GOTO-1 module, do the following:
- stop all activated and created containers in portainer, wait for the processes to finish cleanly,
- ssh into your Duckiebot using the following command. Wait for 30 seconds before plugging out the battery.
`$ ssh DUCKIEBOT_NAME sudo poweroff`


### Important assumptions
This code assumes the following assumptions within the Duckietown environment set-up:
- DT map as hardcoded in `path_planning_class`,
- no other AT's present as the ones hardcoded in `path_planning_class`,
- no varying lighting conditions,
- no external factors (s.a. obstacles on the roads),
- an acceptably functioning `indefinite_navigation` demo version with ROS graph as attached.


## Running GOTO-1
Carefully follow the steps below to implement the proj-goto-1 solution onto your duckiebot.
- [ ] Read the README.md file
- [ ] Scan through the scripts and change the name of the DB if necessary
- [ ] Read the cmd.txt file
- [ ] Add any desired or necessary extensions to your operating system (s.a. dts shell, docker, ...)
- [ ] Execute the cmd.txt file and change all DB dependent parameters if necessary (s.a. IP address and name)



## Troubleshooting
As the existing framework of `indefinite_navigation` is not stable, and issues may arise within the development branch of Duckietown (`daffy`), the following may be of help:
- the scripts for GOTO-1 can overrule the gain and trim values with new values passed through the command terminal:
    - for calibrating `intersection_nagivation` see the file [here](https://github.com/duckietown-ethz/proj-goto-1/blob/master/media/debug_intersection_navigation.pdf)
    - for calibrating `lane_following` see [here](https://github.com/duckietown-ethz/proj-goto-1/blob/master/media/debug_intersection_navigation.pdf)
- if there is a persisting tendency for the Duckiebot to not read out the correct AT at an intersection:
    - intervene using the joystick controller
    - take out non-intersection sign AT's (s.a. STOP, ROAD_NAME, ...)
- other

