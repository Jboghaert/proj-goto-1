<div figure-id="fig:header">
     <img src="media/header.png" style='width: 20em'/>
</div>


# Goal & Description
<!--{#goto_1_description}-->
Author: [J. Boghaert](https://github.com/Jboghaert)

Tutors: [M. Hosner](https://github.com/hosnerm), [G. Zardini](https://github.com/gzardini)

The goal of this project is to **navigate** a single Duckiebot within a predefined Duckietown lay-out from **any starting point A** to a randomly generated **arrival point B**. The Duckiebot is - with GOTO-1 - able to start driving and autonomously navigate through Duckietown in such way that it uses the existing infrastructure only to localize and navigate, that it follows the shortest path possible and that it reaches the desired arrival point with an acceptable accuracy. This means that GOTO-1 uses existing, standard infrastructure as landmarks to locate itself within the city, and that it uses these landmarks (i.e. apriltags) as nodes to calculate the shortest path within the predefined Dijkstra graph representing Duckietown. More information (also on the GOTO-1 pipeline architecture) can be found in the [final report](https://drive.google.com/file/d/16wffD6FrJ81WGrtCKoku1a_nmQv3DsbB/view) or the [final presentation](https://drive.google.com/file/d/14vOtb7f9E6BTxhA964dL-dLqI8VofHpZ/view).

In general, the GOTO-1 package tackles the three subproblems of global localization, respectively localization/path planning, navigation and the last mile problem (arrival accuracy and stopping). In order to implement this package into an executable code, the entire GOTO-1 system required for the mission of global localization is built upon the existing `indefinite navigation` framework already available [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/demo_indefinite_navigation.html).

<!-- Pipeline description
The driving input to GOTO-1 are the intersection AT's, as these provide a reliable (a certain AT will never return another AT id) and robust way to locate the Duckiebot in the city. This firstly originated from the need to have a certain landmark - mapped within a predefined map - that could serve as a localization tool. These then become - by extension - also the nodes for the subsequent path planning. In especially since the predefined map would already require to have all AT's mapped in order to allow localization from all possible starting points within the map. Also, using the AT's satisfies the constraint of not making U-turns within Duckietown. As a result, at each intersection an AT is read out and benchmarked/validated against the generated sequence of nodes composing the shortest path (note: the red stoplines at intersections could provide the trigger function for turn commands as well, but these proved to be less reliable during testing within the `indefinite_navigation` framework.) The in parallel to the node sequence generated sequence of turn commands then publishes the turn command to the `unicorn_intersection` node, which is responsible for the execution of intersection navigation and control. Once the final AT is encountered, the final turn command is passed and a state feedback loop takes over to go the last mile to the desired arrival point B. It passes a stop command once the distance from the last intersection (in number of midline stripes) is met.
-->


<!-- See final report
# Content & pipeline structure {#goto_1_pipeline}
Within the `packages/my_package/src` directory, all nodes and external classes for the GOTO-1 project can be found. The figure below shows the overall pipeline of the project. It can be seen that the altered `indefinite_navigation` module is running all the time. In addition, the joystick controller is used to trigger and overrule the GOTO-1 modules whenever necessary. A full description of the pipeline can again be found in the [final report](https://drive.google.com/file/d/16wffD6FrJ81WGrtCKoku1a_nmQv3DsbB/view).

<div figure-id="fig:pipeline_vis">
     <img src="media/pipeline_vis.png" style='width: 20em'/>
</div>

**Legenda:** Unmarked inputs are given or self-determined, yellow blocks are running in the existing framework of `indefinite_navigation`, and orange/grey-marked items represent the custom blocks developed for GOTO-1.

## 1. Input parameters:
Upon launching GOTO-1, the final arrival point B needs te be defined. In particular, two parameters have to be passed by the terminal, respectively defining the final lane and a point (distance) within that lane the Duckiebot should be in upon arrival:
- `goal_input`: of type `tag_id`, this defines the final lane of the arrival point / the final lane the DB should be in upon Shutdown by specifying the AT id that is encountered at the end of that lane,
- `goal_distance`: of type `int`, this specifies the distance between the last intersection the DB passes and the arrival point B.

All other values that can be passed from the terminal are tuning values to finetune the DB behaviour during `indefinite_navigation`. Default values for these parameters are defined in the `proj_goto_1.launch` [file](https://github.com/duckietown-ethz/proj-goto-1/blob/master/packages/my_package/launch/proj_goto_1.launch) of GOTO-1.

Another necessary input that should be included - and be linked to your Duckietown configuration - is the Dijkstra graph representation of your Duckietown as well as a graph/mapping of the possible turn commands between nodes (AT's). A more thorough explanation is given in the GOTO-1 [report](https://drive.google.com/file/d/16wffD6FrJ81WGrtCKoku1a_nmQv3DsbB/view). See also the `path_planning_class` to see the implementation of this graph.

## 2. global_localization_node:
This node **localizes** the duckiebot and uses an external path planning class to generate the shortest path to get from the localized point to a given destination point. It is the main code of GOTO-1 and passes the desired turn commands per intersection, as well as the stop command upon arrival - i.e. it **navigates** through Duckietown. The driving input of this code are the intersection AT's, serving as nodes for the Dijkstra graph within the `path_planning_class` outlined next. Upon arrival, the node publishes a **stop command** and shuts down the GOTO-1 package.

**Note:**
- The code itself explains in- and output arguments, as well as additional, more detailed information on the exact approach and reasoning behind the code.
- The code currently features a switch `self.se_switch` that is *false* by default, in order to change between two state estimators (i.e. the state_estimation_node as explained next or a simple feedforward timer that calculates the time between last intersection and arrival point based on the current velocity parameters). The value of this switch can be changed upon running the node or during run-time from the terminal using `# rosparam set`. Note however that the feedforward timer is not yet part of the package in this repository.

## 3. path_planning_class:
This class is imported by `localization_node` and calculates the **shortest path** (SP) given an input and output node within the predefined DT map. The predefined DT map is hardcoded in this class, and should be adapted to the actual Duckietown you want to use. For the path planning itself, the class uses a Dijkstra algorithm to define the shortest path. It then gives a sequence of AT's (nodes) to define this path, as well as the corresponding turn commands it should execute in order to navigate/perform the shortest path.

## 4. state_estimation_node:
This node executes the **last mile** problem of proj-goto-1 by converting the input distance (from a certain AT) to passing a desired number of midline stripes and visually counting these until the desired position is reached.
-->


# Global Localization Demo {#global_localization_demo}

## 1. Teaser
A successful run of the GOTO-1 demo version can be found [here](https://drive.google.com/file/d/1ceo435i2H9kbQmCQbiqCNoKmAQx5jAJe/view) for localization, path planning and navigation of the Duckiebot, and [here](https://drive.google.com/file/d/1__jHM4iRiDjxXo_UnNaNH6fftmc-62mf/view) for navigation, state estimation and shutdown procedure of the GOTO-1 project.


## 2. Duckietown set-up notes
For the GOTO-1 project, any Duckietown configuration can be used that adheres to the [specifications](https://docs.duckietown.org/daffy/opmanual_duckietown/out/dt_ops_appearance_specifications.html). However, certain additional limitations were made in order for the localization, planning and execution of the *global localization* solution to make sense. These can be found under [Prerequisites and assumptions](#goto_1_implementation).

<!-- GOTO-1 restrictions/limitations
- the AT density within Duckietown should stay within reasonable limits and should ideally not exceed the already implemented AT's (s.a. stop signs, intersection signs, streetnames, ...)
- no visual marks - other than the ones outlined in the [appearance specifications](https://docs.duckietown.org/daffy/opmanual_duckietown/out/dt_ops_appearance_specifications.html) - can be used for initial localization, navigation and stopping of the Duckiebot
- no U-turns are allowed within Duckietown - basic traffic rules should be taken into account
- the Duckiebot should stay within the lanes, and should have the correct orientation (right lane driving direction)
-->


## 3. Demo pre-flight checklist
A quick pre-flight checklist for running the GOTO-1 demo is provided below:
- Make sure all assumptions and restrictions for GOTO-1 as explained in [Prerequisites and assumptions](#goto_1_implementation) are met.
    * Set up the Duckiebot as explained in section [Prerequisites and assumptions](#goto_1_implementation).
    * Set up a correctly configured Duckietown as explained in section [Prerequisites and assumptions](#goto_1_implementation), and make sure the DT map is configured as in `path_planning_class` (more info can be found in the [final report](https://drive.google.com/file/d/16wffD6FrJ81WGrtCKoku1a_nmQv3DsbB/view)).
- Set up the altered framework of the `indefinite navigation` demo on which GOTO-1 will be built, as further explained in section [Setting up the framework](#goto_1_implementation).
- Check if the activated Docker containers are visible in Portainer, accessed via *DUCKIEBOT_NAME.local:9000* (repeat this check after every newly run Docker container). Check if the containers are up and running by checking their log.
- Implement the GOTO-1 functionality on top of `indefinite navigation` as further explained in section [Implementing GOTO-1](#goto_1_implementation).
    * Make sure to define the input parameters correctly. I.e. `goal_input` as the AT defining the lane (the AT itself will not be read by the Duckiebot in normal circumstances) and `goal_distance` as distance between final intersection and arrival point in cm.
    * Check whether the ROS graph of the entire GOTO-1 module is correct using `rqt_graph` and use `rqt_image_view` to check whether the camera functions properly (further explained in section [Additional](#goto_1_implementation)).
- Run the demo using the joystick controller, as explained in section [Implementing GOTO-1](#goto_1_implementation).


## 4. Demo instructions
<!--{#goto_1_implementation}-->
The scripts within the GOTO-1 project are written for the 2019 Duckietown (AMOD) class at ETH Zürich. The entire project is based on a ROS-template providing a boilerplate repository for developing ROS-based software in Duckietown, to be found [here](https://github.com/duckietown/template-ros). Throughout this document `$ some_command` refers to a command from the terminal within the project directory, and `# some_command` refers to a command within the root of a Docker container (accessed using `/bin/bash`).

Running the project should be implemented in the existing framework of `indefinite_navigation`, more info to be found [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/demo_indefinite_navigation.html) with the default rosgraph to be found [here](https://github.com/duckietown-ethz/proj-goto-1/blob/master/media/indefinite_navigation_default_rosgraph.png). This framework allows us to comply with the Duckietown traffic rules, lane following and the necessary task prioritization of incoming commands. The implementation of the GOTO-1 project requires some changes to be made within the indefinite navigation framework, which are outlined in the next sections.


### 4.1 Prerequisites and assumptions:
The GOTO-1 package assumes the following assumptions within the Duckietown environment set-up:
- DT map as hardcoded in `path_planning_class` (the demo example can be found [here](https://github.com/duckietown-ethz/proj-goto-1/blob/master/media/lab_dt_map.png)),
- no varying lighting conditions,
- no external factors (s.a. obstacles on the roads),
- an acceptably functioning `indefinite_navigation` demo version with ROS graph as attached.

In addition, in order to function properly and start using the ([daffy](https://docs.duckietown.org/daffy/index.html)) Duckietown-environment in the first place, GOTO-1 requires the following:
- a well calibrated Duckiebot, i.e. using [wheel calibration](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/wheel_calibration.html) and [camera calibration](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/camera_calib.html) (more [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/index.html)),
- a well set-up laptop (preferably Ubuntu), further explained [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/laptop_setup.html),
- a well established connection between Duckiebot and (any) desktop, further explained [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/setup_duckiebot.html),
- a correctly set up Duckietown configuration (more info [here](https://docs.duckietown.org/daffy/opmanual_duckietown/out/dt_ops_assembly.html)).


### 4.2 Setting up the framework:
Before building anything, make sure to be connected to your Duckiebot, and retrieve its IP address through:
~~~~
$ ping DUCKIEBOT_NAME.local
~~~~
Then, make sure to pull the latest docker images for `dt-core`, `dt-car-interface` and `dt-duckiebot-interface` through:
~~~~
$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-car-interface:daffy
$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-duckiebot-interface:daffy
$ docker -H DUCKIEBOT_NAME.local pull duckietown/dt-core:daffy
~~~~

Run the required demo containers (also outlined [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/demo_indefinite_navigation.html)) and make sure no (old) `dt-core`, `dt-car-interface` or `dt-duckiebot-interface` are running. You could also manually start these demo containers from Portainer (accessed through *DUCKIEBOT_NAME.local:9000/#/containers* in a browser) if you already ran them before. Within Portainer, you will also be able to see the logging services of all containers running and inspect the DUCKIEBOT's behaviour. Now, run:
~~~~
$ dts duckiebot demo --demo_name all_drivers --duckiebot_name DUCKIEBOT_NAME --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy
$ dts duckiebot demo --demo_name all --duckiebot_name DUCKIEBOT_NAME --package_name car_interface --image duckietown/dt-car-interface:daffy
~~~~

As mentioned, the demo container for `indefinite_navigation` is altered by excluding the node of `random_april_tag_turns_node`, and - in a next step - replacing it with the GOTO-1 package. The exclusion is done by altering the `indefinite_navigation.launch` file and putting the boolean value of `random_apriltag` to *"false"* as follows:

~~~~
$ docker -H DUCKIEBOT_NAME.local run -it --name dt-core-goto1 -v /data:/data --privileged --rm --net host duckietown/dt-core:daffy /bin/bash
~~~~
Then, once inside the root, navigate to the `packages/duckietown_demos/launch` directory and then install the text editor *vim* (or any other text editor) to change the boolean value within the correct file.
~~~~
# cd packages/duckietown_demos/launch
# apt-get update
# apt-get install vim
# vim indefinite_navigation.launch
~~~~
Once inside the file, press *"i"* to edit, and `esc` followed by *":wq"* to close and save the file. Then, launch the altered `indefinite_navigation`:
~~~~
# roslaunch duckietown_demos indefinite_navigation.launch veh:="DUCKIEBOT_NAME"
~~~~

**Note:** The above procedure of installing vim should be performed every time when preparing the `indefinite_navigation` framework for the GOTO-1 implementation. An alternative approach was opted in section [Future Improvements](#goto_1_improvements), but not yet implemented.

**Important:** Keep the demo containers running at all times, and allow the containers enough time (about 3 minutes) to be up and running. Use a new terminal window for the next section(s).


### 4.3 Implementing GOTO-1:

Once the framework is set up, build the image:
~~~~
$ dts devel watchtower stop -H DUCKIEBOT_NAME.local
$ chmod +x ./packages/my_package/src/localization_node.py
$ chmod +x ./packages/my_package/src/state_estimation.py
$ dts devel build -f --arch arm32v7 -H DUCKIEBOT_NAME.local
~~~~
Then run the GOTO-1 module, and access its root to pass the desired input commands:
~~~~
$ docker -H DUCKIEBOT_NAME.local run -it --name proj-goto-1 --privileged -v /data:/data -e ROS_MASTER_URI=http://DUCKIEBOT_IP:11311/ --rm --net host duckietown/IMAGE_NAME:IMAGE_TAG /bin/bash
# roslaunch my_package proj_goto_1.launch goal_input:="199" goal_distance:="40" new_v_bar:="0.5" inter_nav_ff_left:="0.4" inter_nav_ff_right:="-0.6" inter_nav_time_left_turn:="3.2" inter_nav_time_right_turn:="1.5"
~~~~

Start the demo as follows, from another terminal:
~~~~
$ dts duckiebot keyboard_control DUCKIEBOT_NAME
~~~~

Once up and running, your ROS graph should display something like the image below.

**Note:** All values have been assigned default values as defined in the `proj_goto_1.launch` file [here](https://github.com/duckietown-ethz/proj-goto-1/blob/master/packages/my_package/launch/proj_goto_1.launch). Although these give some useful behaviour and you could leave them out from the command, you are encouraged to find the most optimal trim values for your Duckiebot yourself.

<div figure-id="ros_nodes">
     <img src="media/ros_nodes.png" style='width: 20em'/>
</div>


### 4.4 Stopping procedure:
When stopping the GOTO-1 module, do the following:
- close all auxiliary terminals you might have used (s.a. joystick control, start_gui_tools, rviz, ...),
- stop all activated and created containers in portainer (accessed through *DUCKIEBOT_NAME.local:9000/#/containers* in a browser), wait for the processes to finish cleanly,
- ssh into your Duckiebot using the following command. Wait for 30 seconds before plugging out the battery.
`$ ssh DUCKIEBOT_NAME sudo poweroff`


### 4.5 Additional functionalities:
The following packages can be of further help to analyze (any) node or node-system:

In order to see the ROS graph, to see what your Duckiebot sees or to use any rqt functionalities:
~~~~
$ dts start_gui_tools DUCKIEBOT_NAME
# rostopic list
# rosparam list
# rqt
# rqt_graph
# rqt_image_view
~~~~

In order to dynamically adjust parameters (during run-time), start another container and pass parameters using /bin/bash as follows:
~~~~
$ docker run -it --rm  -e ROS_MASTER_URI="http://DUCKIEBOT_IP:11311/" duckietown/dt-ros-commons:daffy-amd64 /bin/bash
# rosparam list
# rosparam get /directory/path/file
# rosparam set /directory/path/file DESIRED_VALUE
~~~~

<!--IGNORE

# Running GOTO-1
Carefully follow the steps below to implement the proj-goto-1 solution onto your duckiebot.
- [ ] Read the README.md file
- [ ] Scan through the scripts and change the name of the DB if necessary
- [ ] Read the cmd.txt file
- [ ] Add any desired or necessary extensions to your operating system (s.a. dts shell, docker, ...)
- [ ] Execute the cmd.txt file and change all DB dependent parameters if necessary (s.a. IP address and name)

UNTIL HERE-->


## 5. Troubleshooting
<!--{#goto_1_troubleshooting}-->
The existing framework of `indefinite_navigation` was at the time of testing not stable, and issues may arise within the development branch of Duckietown (`daffy`). Example videos of that can be found [here](https://drive.google.com/file/d/1UeRevwjQu62ARu0INYo6YaVFOHmZ3tOM/view) for AT detection where the Duckiebot is supposed to go straight but reads out an incorrectly oriented AT, [here](https://drive.google.com/file/d/1y1i8eiXv-RWP5j5Na8e2t8xoGfEP4eeL/view) for intersection navigation where the Duckiebot is supposed to take a left turn and [here](https://drive.google.com/file/d/1UkE1kg3MPbjpTgVxW5ECx_uzblhUJiCY/view) for lane following. In those cases, the following may be of help:

> **Symptom:** The Duckiebot does not navigate well at intersections (overshoot) and/or does not follow the lane smoothly:

**Resolution:** the scripts for GOTO-1 can overrule the gain and trim values with new values passed through the command terminal (or through the root of another running Docker container):
- for calibrating `intersection_nagivation` see the file [here](https://github.com/duckietown-ethz/proj-goto-1/blob/master/media/debug_intersection_navigation.pdf)
- for calibrating `lane_following` see [here](https://github.com/duckietown-ethz/proj-goto-1/blob/master/media/debug_intersection_navigation.pdf)
- **note**: the `kinematics_node/gain` parameter affects both linear and angular velocity, so preferably do not use this when you want to tune only the linear velocity


> **Symptom:** There is a persisting tendency for the Duckiebot to not read out the correctly oriented AT at an intersection:

**Resolution:** Intervene manually or simplify the Duckietown lay-out and other restrictions:
- intervene using the joystick controller
- take out non-intersection sign AT's (s.a. STOP, ROAD_NAME, ...) such that there are no other AT's present as the ones hardcoded in `path_planning_class`


> **Symptom:** State estimation does not function properly or a low accuracy of state feedback is encountered:

**Resolution:** This requires finetuning of several parameters, but should be stable once a value exhibits well-functioning behaviour:
- tune the camera frame rate in the `state_estimation` node (minimal values are given, but could be increased in order to be more precise in detecting midline stripe discontinuities)
- tune the rospy rate used by the `global_localization` node and the `state_estimation` node to obtain an optimal processing of incoming camera images and passing of state feedback
- tune the linear velocity `v_bar` of the `lane control` node (this can be passed upon starting the GOTO-1 package or dynamically via the `# rosparam set` command)
- **note**: changing the above values is likely to have an influence on `lane_following` as well, for which other values might need to be tuned as well (see the first symptom).


> **Symptom:** The set-up of the Duckiebot is incorrect (s.a. network problems, lost connection, blinking, ...) or more general problems with the `indefinite_navigation` framework:

**Resolution:**
- issues regarding the set-up of your Duckiebot: [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/setup_troubleshooting.html#part:setup-troubleshooting)
- issues regarding the use of the `indefinite_navigation` framework: [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/trouble_unicorn_intersection.html)

In general, sufficient time should be spent for tuning the parameter values of the Duckiebot. Note however, that a single success for a certain set of parameter values does not necessarily mean the values have converged and are optimal for all other trials.


# Global Localization improvements
<!--{#goto_1_improvements}-->
As for any project, there are certain aspects of the GOTO-1 package and the involved framework of `indefinite_navigation` that can be improved. In especially, the following submodules could benefit from the following:
- the `apriltag_detection` could make use of the AT pose in order to filter out only the correctly oriented AT's (as AT's parallel to the line of sight currently can be favoured over the ones that are perpendicular to the line of sight),
- the `state_estimation` module and its accuracy could be improved by increasing the rate of analyzed frames, and lowering the upper bound for linear velocity (note that the latter also requires to finetune the other `lane_following` parameters,
- the `intersection_navigation` parameters are currently passed as a feedforward command, and are not tailored a specific Duckiebot (developing a better feedback-based intersection navigation is currently part of another project),
- the `lane_control` parameters are currently very unstable, and are again not tailored to a specific Duckiebot (developing a better adaptive lane control module is currently part of another project as well). During the development of GOTO-1, it was opted to do the next substeps. These should then allow to build an updated version of the dt-core, with the improved modules from the adaptive lane control project and the `random_apriltag` parameter as *false*.
    - permanently fork the `dt-core` repository,
    - change the base image of the GOTO-1 Dockerfile to dt-core (you want to build upon this image in order to be able to alter it, rather than the default dt-ros-commons image)
    - attach the `lane_control` and `line_detector` packages developed in the `lane_control` [project](https://github.com/duckietown-ethz/proj-lf-adaptive) (AMOD 2019) by S. Arreghini and P. Griffa to the GOTO-1 repository,
    - attach and tailor the .launch files for dt-core by again putting the value of `random_apriltag` to *false*,
    - execute this .launch file by including it in the original proj-goto-1.launch file,
    - **note**: this alternative was in the end not implemented, as the timeframe at the remaining time of development was limited and the (stable) implementation was only proved for `lane_following` and not yet for the larger `indefinite_navigation` module.
- implementation of `duckietown-world` [repo](https://github.com/duckietown/duckietown-world) as basis for the Dijkstra graph, in order to switch more dynamically between various Duckietown configurations. For that, a mapping from the position of the AT's in the `dt-world` format to the format of the Dijkstra graph in the `path_planning_class` should be build.
