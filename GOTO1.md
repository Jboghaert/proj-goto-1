<div figure-id="fig:header">
     <img src="media/header.png" style='width: 20em'/>
</div>


# Goal & Description {#goto_1_description}
Author: [J. Boghaert](https://github.com/Jboghaert)

Tutors: [M. Hosner](https://github.com/hosnerm), [G. Zardini](https://github.com/gzardini)

The goal of this project is to **navigate** a single Duckiebot within a predefined Duckietown lay-out from **any starting point A** to a randomly generated **arrival point B**. The Duckiebot is - with GOTO-1 - able to start driving and autonomously navigate through Duckietown in such way that it uses the existing infrastructure only to localize and navigate, that it follows the shortest path possible and that it reaches the desired arrival point with an acceptable accuracy. This means that GOTO-1 uses existing, standard infrastructure as landmarks to locate itself within the city, and that it uses these landmarks (i.e. apriltags) as nodes to calculate the shortest path within the predefined Dijkstra graph representing Duckietown. More information (also on the GOTO-1 pipeline architecture) can be found in the [final report](https://drive.google.com/file/d/16wffD6FrJ81WGrtCKoku1a_nmQv3DsbB/view) or the [final presentation](https://drive.google.com/file/d/14vOtb7f9E6BTxhA964dL-dLqI8VofHpZ/view).

In general, the GOTO-1 package tackles the three subproblems of global localization, respectively localization/path planning, navigation and the last mile problem (arrival accuracy and stopping). In order to implement this package into an executable code, the entire GOTO-1 system required for the mission of global localization is built upon the existing `indefinite_navigation` framework already available [here](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/demo_indefinite_navigation.html).

<!-- Pipeline description
The driving input to GOTO-1 are the intersection AT's, as these provide a reliable (a certain AT will never return another AT id) and robust way to locate the Duckiebot in the city. This firstly originated from the need to have a certain landmark - mapped within a predefined map - that could serve as a localization tool. These then become - by extension - also the nodes for the subsequent path planning. In especially since the predefined map would already require to have all AT's mapped in order to allow localization from all possible starting points within the map. Also, using the AT's satisfies the constraint of not making U-turns within Duckietown. As a result, at each intersection an AT is read out and benchmarked/validated against the generated sequence of nodes composing the shortest path (note: the red stoplines at intersections could provide the trigger function for turn commands as well, but these proved to be less reliable during testing within the `indefinite_navigation` framework.) The in parallel to the node sequence generated sequence of turn commands then publishes the turn command to the `unicorn_intersection` node, which is responsible for the execution of intersection navigation and control. Once the final AT is encountered, the final turn command is passed and a state feedback loop takes over to go the last mile to the desired arrival point B. It passes a stop command once the distance from the last intersection (in number of midline stripes) is met.
-->


<!-- See final report
# Content & pipeline structure {#goto_1_pipeline}
-->

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


