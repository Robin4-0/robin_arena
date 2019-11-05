
## Arena Setup of the Autonomous Robotics master course (Dept. Information Engineering, Univ. of Padova)-- A.A. 19/20

### Project: Human-robots cooperation for assembly tasks in Industry 4.0

### Introduction

Factories of Industry 4.0 will be populated by human operators and robots able to cooperate and collaborate in order to fulfill complex, non-repetitive tasks. An example is the storage of goods and their assembly: mobile manipulator robots will help humans in retrieving these goods and transporting them within the factory warehouse. Thus, they will autonomously navigate without the need of following electrified paths or lines drawn on the ground. They will autonomously compute the optimal path from their current pose to a target one while avoiding fixed or moving obstacles. They will recognize and manipulate those goods that need to be transported.

### Project description 

Suppose to have a set of *N* goods in your warehouse, each of them uniquely identified. *x* of them are required to fulfill an assembly task. Thus, suppose to have a subset of *n ≤ N* goods on a table. A human operator asks for *x ≤ n* of them, according to the desired assembly task. A camera is mounted on that table, together with a UR5 manipulator robot, equipped with a Robotiq 3-Finger Adaptive Gripper. This visual sensor detects the required objects and publishes their poses (position and orientation). The UR5 exploits such a data to pick up every required object and place it on the top of a Marrtino mobile robot, previously docked near the manipulator. Marrtino brings the pieces to an unloading station where a human oparator picks up the objects and assembles them. During its path, the mobile robot faces a narrow passage, delimited by fixed barriers, and an open space area, populated by fixed and movable obstacles.  


### Useful commands 

> *Arena bringup*:

```sh
$ roslaunch ar_arena ar_bringup.launch simulation:=BOOL
```
where *BOOL == true* if the simulated environment is launched; otherwise, *BOOL == false*. This command launches robots (UR5 - with gripper - and Marrtino), Kinect, and all the motion stacks necessary to move the robots (MoveIt! - for the manipulator robot - and the ROS Navigation Stack - for the mobile robot). Please open RViz to visualize the scene and try the manipulation and navigation stacks. 

Remember that every robot has a namaspace: by default, */ur5* for the manipulator robot and */marrtino* for the mobile robot.

> *Apriltag*:

```sh
$ roslaunch ar_arena apriltag.launch simulation:= BOOL
```
where *BOOL* is defined as before.

> *Robotiq 3-Finger Adaptive Gripper*:

- Activation:

```sh
$ rostopic pub --once /left_hand/command robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput "{rACT: 1, rMOD: 0, rGTO: 0, rATR: 0, rGLV: 0, rICF: 0, rICS: 0, rPRA: 0, rSPA: 0, rFRA: 0, rPRB: 0, rSPB: 0, rFRB: 0, rPRC: 0, rSPC: 0, rFRC: 0, rPRS: 0, rSPS: 0, rFRS: 0}"
```

- Deactivation:

```sh
$ rostopic pub --once /left_hand/command robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput "{rACT: 0, rMOD: 0, rGTO: 0, rATR: 0, rGLV: 0, rICF: 0, rICS: 0, rPRA: 0, rSPA: 0, rFRA: 0, rPRB: 0, rSPB: 0, rFRB: 0, rPRC: 0, rSPC: 0, rFRC: 0, rPRS: 0, rSPS: 0, rFRS: 0}"
```

- Close:

```sh
$ rostopic pub --once /left_hand/command robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput "{rACT: 1, rMOD: 0, rGTO: 1, rATR: 0, rGLV: 0, rICF: 0, rICS: 0, rPRA: 250, rSPA: 200, rFRA: 200, rPRB: 0, rSPB: 0, rFRB: 0, rPRC: 0, rSPC: 0, rFRC: 0, rPRS: 0, rSPS: 0, rFRS: 0}"
```

- Keep close pose:

```sh
$ rostopic pub --once /left_hand/command robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput "{rACT: 1, rMOD: 0, rGTO: 0, rATR: 0, rGLV: 0, rICF: 0, rICS: 0, rPRA: 0, rSPA: 0, rFRA: 0, rPRB: 0, rSPB: 0, rFRB: 0, rPRC: 0, rSPC: 0, rFRC: 0, rPRS: 0, rSPS: 0, rFRS: 0}"
```
 
- Open:

```sh
$ rostopic pub --once /left_hand/command robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput "{rACT: 1, rMOD: 0, rGTO: 1, rATR: 0, rGLV: 0, rICF: 0, rICS: 0, rPRA: 0, rSPA: 200, rFRA: 0, rPRB: 0, rSPB: 0, rFRB: 0, rPRC: 0, rSPC: 0, rFRC: 0, rPRS: 0, rSPS: 0, rFRS: 0}"
```

- Keep open pose:

```sh
$ rostopic pub --once /left_hand/command robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput "{rACT: 1, rMOD: 0, rGTO: 0, rATR: 0, rGLV: 0, rICF: 0, rICS: 0, rPRA: 250, rSPA: 0, rFRA: 200, rPRB: 0, rSPB: 0, rFRB: 0, rPRC: 0, rSPC: 0, rFRC: 0, rPRS: 0, rSPS: 0, rFRS: 0}"
```


