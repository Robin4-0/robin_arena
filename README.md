
# RobIn 4.0 - Robotics for Industry 4.0

### Introduction

Factories of Industry 4.0 will be populated by human operators and robots able to cooperate and collaborate in order to fulfill complex, non-repetitive tasks. An example is the storage of goods and their assembly: mobile manipulator robots will help humans in retrieving these goods and transporting them within the factory warehouse. Thus, they will autonomously navigate without the need of following electrified paths or lines drawn on the ground. They will autonomously compute the optimal path from their current pose to a target one while avoiding fixed or moving obstacles. They will recognize and manipulate those goods that need to be transported.

### Project description 

Suppose to have a set of *N* goods in your warehouse, each of them uniquely identified. *x* of them are required to fulfill an assembly task. Thus, suppose to have a subset of *n ≤ N* goods on a table. A human operator asks for *x ≤ n* of them, according to the desired assembly task. A camera is mounted on that table, together with a UR5 manipulator robot, equipped with a Robotiq 3-Finger Adaptive Gripper. This visual sensor detects the required objects and publishes their poses (position and orientation). The UR5 exploits such a data to pick up every required object and place it on the top of a Marrtino mobile robot, previously docked near the manipulator. Marrtino brings the pieces to an unloading station where a human oparator picks up the objects and assembles them. During its path, the mobile robot faces a narrow passage, delimited by fixed barriers, and an open space area, populated by fixed and movable obstacles.  


### Arena bringup

```sh
$ roslaunch robin_arena robin_bringup.launch simulation:=BOOL_SIM gripper_enable:=BOOL_GRIPPER spawn_marrtino:=BOOL_MARRTINO arena_name:=ARENA_NAME
```
where *BOOL_SIM == true* if the simulated environment is launched; otherwise, *BOOL_SIM == false*. 
If *BOOL_GRIPPER == true*, the command launches UR5 with a Robotiq 3-finger gripper attached on its end-effector; otherwise, *BOOL_GRIPPER == false* launches UR5 with a magnet. The command also launches Marrtino, Kinect, and the motion stacks necessary to move the manipulator robot. Please open RViz to visualize the scene and try the manipulation and navigation stacks.
If *BOOL_MARRTINO == false* the Marrtino robot will not be shown in the simulated/real arena.
*ARENA_NAME* defines the name of the arena, it can assume the following values only: *robin_arena* or *robin_arena_simplified*. 

Remember that every robot has a namaspace: by default, */ur5* for the manipulator robot and */marrtino* for the mobile robot.

### Apriltag


```sh
$ roslaunch robin_arena apriltag.launch simulation:= BOOL
```
where *BOOL* is defined as before.

### Attach/detach objects (e.g., fake magnet)

- Attach object:

```sh
$ rosservice call /link_attacher_node/attach "model_name_1: 'ROBOT_NAME'
link_name_1: 'ROBOT_LINK_NAME'
model_name_2: 'OBJECT_NAME'
link_name_2: 'OBJECT_LINK_NAME'"
```

where *ROBOT_NAME*, *ROBOT_LINK_NAME*, *OBJECT_NAME*, and *OBJECT_LINK_NAME* are the GAZEBO names of robot and object that has to be attached - and their links. E.g., if the user wants to attach the UR5 robot and the third simulated cube: *ROBOT_NAME = robin_ur5*, *ROBOT_LINK_NAME = wrist_3_link*, *OBJECT_NAME = cube3*, and *OBJECT_LINK_NAME = cube3_link*. 

- Detach object:

```sh
$ rosservice call /link_attacher_node/detach "model_name_1: 'ROBOT_NAME'
link_name_1: 'ROBOT_LINK_NAME'
model_name_2: 'OBJECT_NAME'
link_name_2: 'OBJECT_LINK_NAME'"
```
where names are defined as before.

### MoveIt! - Manipulation

The MoveIt! packages of both the UR5 equipped with the 3-Finger adaptive gripper and the UR5 equipped with the magnet are already launched during the RobIn arena bringup. 

*Note*: as two different robots populate the environment, different namespaces are required. Focusing on the manipulator, the following Rviz setup is needed: 
- Move Group Namespace: *ur5*;
- Robot Description: *ur5/robot_description*;
- Planning Group: *manipulator*.

*See the Rviz configuration file*

### Navigation

Launch marrtino navigation stack:

```sh
$ roslaunch marrtino_navigation robot_navigation.launch map_file:=MAP scan_topic:=/marrtino/scan
```
with *MAP* the path to */robin_arena/maps/arena.yaml*.

*Note*: As for the UR5 manipulator robot, Marrtino has its own namespace. Thus, in Rviz:
- Robot Model/Robot Description: */marrtino/robot_description*;

and in the Panel/Tool Properties of Rviz:
- 2D Pose Estimate/Topic: */marrtino/initial_pose*;
- 2D Nav Goal/Topic: */marrtino/move_base_simple/goal*;
- Publish Point/Topic: */marrtino/clicked_point*

*See the Rviz configuration file*

### Rviz

```sh
$ rviz
```

A configuration file is available to facilitate the arena visualization. From the visualizer menu, open */robin_arena/cfg/arena_config.rviz*.

### References

- Tosello E., Castaman N., Tagliapietra L., Menegatti E. *RobIn 4.0: a Modular and Open-Source Robotics Program to Train Future Engineers for Industry 4.0*.  IEEE Transactions on Learning Technologies. *Under review*

- Tosello E., Castaman N., Menegatti E. *Using robotics to train students for Industry 4.0*. 12th IFAC Symposium on Advances in Control Education 2019. July 7-9, Philadelphia, PA, USA. In: IFAC-PapersOnLine. Vol 52. Issue 9. 2019. Pages 153-158. ISSN 2405-8963. https://doi.org/10.1016/j.ifacol.2019.08.185.

- Tosello E., Castaman N., Michieletto S., Menegatti E. *Teaching Robot Programming for Industry 4.0*. In: Moro M., Alimisis D., Iocchi L. (eds) Educational Robotics in the Context of the Maker Movement. Edurobotics 2018. Advances in Intelligent Systems and Computing, vol 946. Springer, Cham. https://doi.org/10.1007/978-3-030-18141-3_9


