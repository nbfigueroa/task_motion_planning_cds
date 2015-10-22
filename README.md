# task_motion_planning_cds
This repository includes the packages and instructions to run the LASA Motion planning architecture developed initially for the rolling task within the Robohow project, but can be used for any task and any type of controller, it only has to output the desired command task spacce (i.e. desired cartesian pose/ft/stiffness)

##Youtube Link below (click on image):
<div style="text-align:center">
[![Task and Motion Planning using CDS Architecture used to Roll Pizza Dough](http://img.youtube.com/vi/br5PM9r91Fg/0.jpg)](http://www.youtube.com/watch?v=br5PM9r91Fg)

This research was conducted in the Learning Algorithms and Systems Laboratory (LASA) at the Swiss Federal Institute of Technology in Lausanne (EPFL) under the supervision of Prof. Aude Billard.  ---- http://lasa.epfl.ch/

It was funded by the EU Project ROBOHOW.COG. ----https://robohow.eu/

###Installation Instructions:

- Install robot-toolkit:
```
$ git clone https://github.com/epfl-lasa/robot-toolkit.git
```

- Install kuka_interface_packages:
```
$ git clone https://github.com/nbfigueroa/kuka_interface_packages.git
```
and don't forget to install motion_generators 
```
$ git clone https://github.com/epfl-lasa/motion-generators.git
```

- Install kuka-rviz-simulation:
```
$ git clone https://github.com/epfl-lasa/kuka-rviz-simulation.git
```
and don't forget to install all dependencies for package [kuka-rviz-simulation](https://github.com/epfl-lasa/kuka-rviz-simulation).

- Install coupled-dynamical-systems package:

```
$ git clone https://github.com/epfl-lasa/coupled-dynamical-systems.git
```

- Finally, install task_motion_planning_cds package:

```
$ git clone https://github.com/nbfigueroa/task_motion_planning_cds.git
```

###Running a simulation for a Pouring task learned from Demonstration:


###References:

[1] N. Figueroa and A. Billard, “Discovering hierarchical structure in heterogenous and sequential task demonstrations,” In preparation.

[2] A. L. Pais, K. Umezawa, Y. Nakamura, and A. Billard, “Task parametrization using continuous constraints extracted from human demonstrations,” Accepted, IEEE TRO, 2015.

[3] A. Shukla and A. Billard, “Coupled dynamical system based arm-hand grasping model for learning fast adaptation strategies,” Robotics and Autonomous Systems, vol. 60, no. 3, pp. 424 – 440, 2012.
