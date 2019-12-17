<div figure-id="fig:header">
     <img src="media/header.png" style='width: 20em'/>
</div>


# Goal & Description
Author: J. Boghaert

Tutors: M. Hosner, G. Zardini

The goal of this project is to **navigate** a single Duckiebot within a predefined Duckietown lay-out from **any starting point A** to a randomly generated **arrival point B**. The Duckiebot is - with GOTO-1 - able to start driving and autonomously navigate through Duckietown in such way that it uses the existing infrastructure only to localize and navigate, that it follows the shortest path possible and that it reaches the desired arrival point with an acceptable accuracy. This means that GOTO-1 uses existing, standard infrastructure as landmarks to locate itself within the city, and that it uses these landmarks as nodes to calculate the shortest path within the predefined Dijkstra graph representing Duckietown.

The driving input to GOTO-1 are the intersection AT's, as these provide a reliable (a certain AT will never return another AT id) and robust way to locate the Duckiebot in the city. This firstly originated from the need to have a certain landmark - mapped within a predefined map - that could serve as a localization tool. These then become - by extension - also the nodes for the subsequent path planning. In especially since the predefined map would already require to have all AT's mapped in order to allow localization from all possible starting points within the map. Also, using the AT's satisfies the constraint of not making U-turns within Duckietown. As a result, at each intersection an AT is read out and benchmarked/validated against the generated sequence of nodes composing the shortest path (note: the red stoplines at intersections could provide the trigger function for turn commands as well, but these proved to be less reliable during testing within the `indefinite_navigation` framework.) The in parallel to the node sequence generated sequence of turn commands then publishes the turn command to the `unicorn_intersection` node, which is responsible for the execution of intersection navigation and control. Once the final AT is encountered, the final turn command is passed and a state feedback loop takes over to go the last mile to the desired arrival point B. It passes a stop command once the distance from the last intersection (in number of midline stripes) is met.

## Restrictions

# Content & pipeline structure
Within the `packages/my_package/src` directory, all nodes and external classes for the GOTO-1 project can be found. The figure below shows the overall pipeline of the project. It can be seen that the altered `indefinite_navigation` module is running all the time. In addition, the joystick controller is used to trigger and overrule the GOTO-1 modules whenever necessary.

<div figure-id="fig:pipeline_vis">
     <img src="media/pipeline_vis.png" style='width: 20em'/>
</div>

**Legenda:** Unmarked inputs are given or self-determined, yellow blocks are running in the existing framework of `indefinite_navigation`, and orange/grey-marked items represent the custom blocks developed for GOTO-1.

## 1. Input parameters
Upon launching GOTO-1, the final arrival point B needs te be defined. In particular, two parameters have to be passed by the terminal, respectively defining the lane and a point (distance) within that lane:
- `goal_input`: of type `tag_id`, this defines the final lane of the arrival point / the final lane the DB should be in upon Shutdown by specifying the AT id that is encountered at the end of that lane,
- `goal_distance`: of type `int`, this specifies the distance between the last intersection the DB passes and the arrival point B,

All other values that can be passed from the terminal are tuning values to finetune the DB behaviour during `indefinite_navigation`. Default values for these parameters are defined in the `proj_goto_1.launch` [file](https://github.com/duckietown-ethz/proj-goto-1/blob/master/packages/my_package/launch/proj_goto_1.launch) of GOTO-1.

## 2. global_localization_node
This node **localizes** the duckiebot and uses an external path planning class to generate the **shortest path** to get from the localized point to a given destination point. It is the main code of GOTO-1 and passes the desired turn commands per intersection, as well as the stop command upon arrival. The driving input of this code are the intersection AT's, serving as nodes for the Dijkstra graph within the `path_planning_class` outlined next.

**Note:**
The code itself explains in- and output arguments, as well as additional, more detailed information on the exact approach and reasoning behind the code.

## 3. path_planning_class
This class is imported by `localization_node` and calculates the **shortest path** (SP) given an input and output node within the predefined DT map. The predefined DT map is hardcoded in this class, and should be adapted to the actual Duckietown you want to use.

For the path planning, the class uses a Dijkstra algortihm to define. Dijkstra !! No U-turns allowed

## 4. state_estimation_node
This node executes the **last mile** problem of proj-goto-1 by converting the input distance (from a certain AT) to passing a desired number of midline stripes and visually counting these until the desired position is reached.


# Implementation
The scripts within the GOTO-1 project are written for the 2019 Duckietown (AMOD) class at ETH Zürich. The entire project is based on a ROS-template providing a boilerplate repository for developing ROS-based software in Duckietown, to be found [here](https://github.com/duckietown/template-ros).

Running the project should be implemented in the existing framework of `indefinite_navigation`, more info to be found [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/demo_indefinite_navigation.html) with the default rosgraph to be found [here](https://github.com/duckietown-ethz/proj-goto-1/blob/master/media/indefinite_navigation_default_rosgraph.png). This framework allows us to comply with the Duckietown traffic rules, lane following and the necessary task prioritization of incoming commands. The implementation of the GOTO-1 project requires some changes to be made within the indefinite navigation framework, which are outlined in the next sections.


## 1. Prerequisites and assumptions
The GOTO-1 package assumes the following assumptions within the Duckietown environment set-up:
- DT map as hardcoded in `path_planning_class`,
- no other AT's present as the ones hardcoded in `path_planning_class`,
- no varying lighting conditions,
- no external factors (s.a. obstacles on the roads),
- an acceptably functioning `indefinite_navigation` demo version with ROS graph as attached.

In addition, in order to function properly and start using the ([daffy](https://docs.duckietown.org/daffy/index.html)) Duckietown-environment in the first place, GOTO-1 requires the following:
- a well calibrated Duckiebot, i.e. using [wheel calibration](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/wheel_calibration.html) and [camera calibration](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/camera_calib.html),
- a well set-up laptop (preferable Ubuntu), further explained [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/laptop_setup.html),
- a well established connection between Duckiebot and (any) desktop, further explained [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/setup_duckiebot.html).


## 2. Setting up the framework
Before building anything, make sure to be connected to your Duckiebot, and retrieve its IP address through:
```
$ ping DUCKIEBOT_NAME.local
```
Then, make sure to pull the latest docker images for `dt-core`, `dt-car-interface` and `dt-duckiebot-interface` through:
```
$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-car-interface:daffy
$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-duckiebot-interface:daffy
$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-core:daffy
```

As outlined in the command [file](https://github.com/duckietown-ethz/proj-goto-1/blob/master/Cmd.txt) in more detail, the demo container for `indefinite_navigation` is altered by excluding the node of `random_april_tags_turn_node`, and - in a next step - replacing it with the GOTO-1 nodes. The exclusion is done by entering the `indefinite_navigation.launch` file and putting the boolean value for the required node to *"false"*.

```
$ docker -H DUCKIEBOT_NAME.local run -it --name dt-core-goto1 -v /data:/data --privileged --rm --net host duckietown/dt-core:daffy /bin/bash
```
Then, once inside the root, navigate to the `packages/duckietown_demos/launch` directory and then install the text editor *vim* to change the boolean value within the correct file.
```
$ cd packages/duckietown_demos/launch
$ apt-get update
$ apt-get install vim
$ vim indefinite_navigation.launch
```
Once inside the file, press *"i"* to edit, and `esc` followed by *":wq"* to close and save the file. Then, launch the altered `indefinite_navigation`:
```
$ roslaunch duckietown_demos indefinite_navigation.launch veh:="maserati4pgts"
```

**Note:** The above procedure of installing vim should be performed every time when preparing the `indefinite_navigation` framework for the GOTO-1 implementation.

**Important:** Keep the demo containers running at all times, and allow the containers enough time (about 3 minutes) to be up and running. Use a new terminal window for the next section(s).



## 3. Implementing GOTO-1

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
$ roslaunch my_package proj_goto_1.launch goal_input:="199" goal_distance:="40" new_v_bar:="0.5" inter_nav_ff_left:="0.4" inter_nav_ff_right:="-0.6" inter_nav_time_left_turn:="3.2" inter_nav_time_right_turn:="1.5"
```
Once up and running, your ROS graph should display something like the image below.

**Note:** All values have been assigned default values as defined in the `proj_goto_1.launch` file [here](https://github.com/duckietown-ethz/proj-goto-1/blob/master/packages/my_package/launch/proj_goto_1.launch). Although these give some useful behaviour and you could leave them out from the command, you are encouraged to find the most optimal trim values for your Duckiebot yourself.


<div class="row">
  <div class="column">
    <img src="media/ros_nodes.png" alt="ros_nodes" style='width:100%'>
  </div>


## 4. Stopping procedure:
When stopping the GOTO-1 module, do the following:
- close all auxiliary terminals you might have used (s.a. joystick control, start_gui_tools, rviz, ...),
- stop all activated and created containers in portainer (accessed through *"DUCKIEBOT_NAME.local:9000/#/containers"*), wait for the processes to finish cleanly,
- ssh into your Duckiebot using the following command. Wait for 30 seconds before plugging out the battery.
`$ ssh DUCKIEBOT_NAME sudo poweroff`


# Running GOTO-1
Carefully follow the steps below to implement the proj-goto-1 solution onto your duckiebot.
- [ ] Read the README.md file
- [ ] Scan through the scripts and change the name of the DB if necessary
- [ ] Read the cmd.txt file
- [ ] Add any desired or necessary extensions to your operating system (s.a. dts shell, docker, ...)
- [ ] Execute the cmd.txt file and change all DB dependent parameters if necessary (s.a. IP address and name)



# Troubleshooting
As the existing framework of `indefinite_navigation` is not stable, and issues may arise within the development branch of Duckietown (`daffy`), the following may be of help:
- the scripts for GOTO-1 can overrule the gain and trim values with new values passed through the command terminal:
    - for calibrating `intersection_nagivation` see the file [here](https://github.com/duckietown-ethz/proj-goto-1/blob/master/media/debug_intersection_navigation.pdf)
    - for calibrating `lane_following` see [here](https://github.com/duckietown-ethz/proj-goto-1/blob/master/media/debug_intersection_navigation.pdf)
    - **note**: the kinematics_node gain affects both linear and angular velocity, so do not use this to tune linear velocity only
- if there is a persisting tendency for the Duckiebot to not read out the correct AT at an intersection:
    - intervene using the joystick controller
    - take out non-intersection sign AT's (s.a. STOP, ROAD_NAME, ...)

Other helpful links:
- issues regarding the set-up of your Duckiebot: [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/setup_troubleshooting.html#part:setup-troubleshooting)
- issues regarding the use of the `indefinite_navigation` framework: [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/trouble_unicorn_intersection.html)
