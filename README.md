# task_motion_planning_cds
This repository includes the packages and instructions to run the LASA Motion planning architecture developed initially for a pizza dough rolling task within the Robohow project, but can be used for any task and any type of controller that outputs the desired command in task space (i.e. desired cartesian pose/ft/stiffness)

###Video of the architecture in action (click on image):
<div style="text-align:center">
[![Task and Motion Planning using CDS Architecture used to Roll Pizza Dough](http://img.youtube.com/vi/br5PM9r91Fg/0.jpg)](http://www.youtube.com/watch?v=br5PM9r91Fg)

This research was conducted in the Learning Algorithms and Systems Laboratory (LASA) at the Swiss Federal Institute of Technology in Lausanne (EPFL) under the supervision of Prof. Aude Billard.  ---- http://lasa.epfl.ch/

It was funded by the EU Project ROBOHOW.COG. ----https://robohow.eu/

###Modular Architecture Description:
![alt tag](https://cloud.githubusercontent.com/assets/761512/10681561/a4dfd458-792a-11e5-973b-0c196fbf9277.png)

Following a brief run-through of the architecture:

  - **Action  planner:** The  action  planner  tells  theMotion  plannerwhich  learned  action  toexecute and the corresponding attractor obtained from the vision module (or fixed values insimulation).
  - **Vision Module (dough/attractor detectors):** This is the vision module, which detects the attractors for each corresponding action in the   sequence or single action involved in the desired task.  (This module can be substituted by fixed values to facilitate simulation and fast integration of new components.)
  - **Motion planner:** Executes the requested action from the Action planner.This involves commanding the next desired state of the end-effector (pose, force/torque) using the currentstate of the end-effector and the parameters learned for the specifc action (i.e.  parameters of Coupled-Dynamical-System for pose control, parameters of probability distribution functionfor force/torque, stiffness profile) until the attractor/force is reached.
  - **EPFL  Task  Models:** These  are  text  files  that  contain  the  parameters  for  the  learnedaction/motion for each task.
  - **Cartesian to Joint State Transformer:** This module takes the desired end-effector com-mand (pose, force/torque) and converts it to joint velocities and stiffness.
  - **Joint to Cartesian State Estimator:** This module estimates the end-effector pose and force/torque from the joint angle/torques provided by the low-level controller.


###Installation Instructions:

**System Requirements:**

OS: Ubuntu 14.04

ROS compatibility: Indigo

For **each package**, the user needs to do the following:

*Download:*
```
$ cd /catkin_ws/src
$ git clone <remote branch>
```
*Build:*
```
$ cd /catkinws/
$ catkinmake
```
**Package list:** 
  1. Install [robot-toolkit](https://github.com/epfl-lasa/robot-toolkit):
  ```
  $ git clone https://github.com/epfl-lasa/robot-toolkit.git
  ```

  2. Install [kuka_interface_packages](https://github.com/nbfigueroa/kuka_interface_packages):
  ```
  $ git clone https://github.com/nbfigueroa/kuka_interface_packages.git
  ```
  and don't forget to install [motion_generators](https://github.com/epfl-lasa/motion-generators) 
  ```
  $ git clone https://github.com/epfl-lasa/motion-generators.git
  ```

  3. Install [kuka-rviz-simulation](https://github.com/epfl-lasa/kuka-rviz-simulation):
  ```
  $ git clone https://github.com/epfl-lasa/kuka-rviz-simulation.git
  ```
  and don't forget to install all dependencies for this package.

  4. Install [coupled-dynamical-systems](https://github.com/epfl-lasa/coupled-dynamical-systems) package:
  ```
  $ git clone https://github.com/epfl-lasa/coupled-dynamical-systems.git
  ```

  5. Finally, install [task_motion_planning_cds](https://github.com/nbfigueroa/task_motion_planning_cds) package:
  ```
  $ git clone https://github.com/nbfigueroa/task_motion_planning_cds.git
  ```
  
###Running a simulation for a Pouring task learned from Demonstration:

#####Robot Simulator
```
$ roslaunch kuka_lwr_bringup lwr_simulation_viz.launch
```

#####Visualization
```
$ rosrun rviz rviz
```

#####Control/Motion Planning

Joint to Cartesian Estimation
```
$ roslaunch state_transformers joint_to_cart_lasa_pour.launch 
```

Cartesian to Joint Estimation
```
$ roslaunch state_transformers cart_to_joint_lasa_sim_pour.launch 
```

Trajectory Generator
  - For complete trajectory generation (open-loop):
  ```
  $ roslaunch motion_planner lasa_sim_fixed_pouring.launch
  ```
  
  - For online trajectory generation (closed-loop with "simulated" robot controllers):
  ```
  $ roslaunch motion_planner lasa_fixed_pouring.launch
  ```

##### Action Planning  
```
$ rosrun lasa_action_planners pouring_demo_fixed_lasa.py
```
Follow the instructions on the commandline...



###References:

[1] N. Figueroa and A. Billard, “Discovering hierarchical structure in heterogenous and sequential task demonstrations,” In preparation.

[2] A. L. Pais, K. Umezawa, Y. Nakamura, and A. Billard, “Task parametrization using continuous constraints extracted from human demonstrations,” Accepted, IEEE TRO, 2015.

[3] A. Shukla and A. Billard, “Coupled dynamical system based arm-hand grasping model for learning fast adaptation strategies,” Robotics and Autonomous Systems, vol. 60, no. 3, pp. 424 – 440, 2012.
