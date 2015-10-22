# task_motion_planning_cds
This repository includes the packages and instructions to run the LASA Motion planning architecture developed initially for the rolling task within the Robohow project, but can be used for any task and any type of controller, it only has to output the desired command task spacce (i.e. desired cartesian pose/ft/stiffness)

##Youtube Link below (click on image):
<div style="text-align:center">
[![Task and Motion Planning using CDS Architecture used to Roll Pizza Dough](http://img.youtube.com/vi/br5PM9r91Fg/0.jpg)](http://www.youtube.com/watch?v=br5PM9r91Fg)

This research was conducted in the Learning Algorithms and Systems Laboratory (LASA) at the Swiss Federal Institute of Technology in Lausanne (EPFL) under the supervision of Prof. Aude Billard.  ---- http://lasa.epfl.ch/

It was funded by the EU Project ROBOHOW.COG. ----https://robohow.eu/

##Short Description and corresponding publications:
In order to learn a complex task which involves a sequence of primitive motions (atomic actions), such as pizza dough rolling, we must learn and discover a hierarchy of parametrizations for each level of the task. Hence, from successful demonstrations performed by a skilled user through kinesthetic teaching, we propose to extract the different levels of knowledge from the task following three main steps, which are sufficient to parametrize the task plan and motion control of the robot.

The first step involves automatic task segmentation and primitive learning. Here we apply a method proposed by Figueroa and Billard [1] which uses a Bayesian Non-Parametric approach for segmentation and clustering, namely an extension of the Beta Process Hidden Markov Model (BPHMM). This method is used to discover the unique primitive motions in a scale, translation and rotation invariant manner, without any prior knowledge of the task or the number of primitives involved. For the dough rolling task this resulted in a sequence of 3 primitive motions (atomic actions): reach, roll and reach back.

We follow by extracting soft task constraints from thediscovered primitive motions, as proposed by Pais et al. [2]. These constraints represent low-level knowledge about the task. They describe the variables of interest, the reference frame to be used and the proper stiffness modulation for each action in the given task. Applied to the dough rolling task this method determined a position controller as suitable for the reaching/reaching back phases and a hybrid force-position decomposition for the rolling action.

Finally, we learn a set of action models for each primitive motion, which represent the parametrization of a Cartesian impedance controller, used to control the KUKA LWR 7DOF robot arm. Each model is formed by a a coupled dynamical system (CDS) model, introduced by Shukla and Billard [3], which encodes the position and orientation as two autonomous dynamical systems and a coupling function between them. Forces are encoded as a function of the position using a Gaussian Mixture Model (GMM). In the rolling task we only consider the vertical force applied on the dough, as this was identified as one of the tasks soft constraints. The stiffness is computed based on a learned stiffness modulation factor and a base stiffness.

Additionally, we extracted high-level knowledge about the task in the form of a success metric. The rolling task presumed multiple iterations of the sequence of atomic actionsof reaching, rolling, reaching back. The goal was to obtain a round dough, of a given area. Which is correlated to properly positioning the starting and ending points of the rolling action. Since each rolling deformed the dough in the desired direction, at each iteration we positioned the attractors for the reaching and rolling along the small axis (second principal component) of a fitted ellipse on the dough.

##REFERENCES

[1] N. Figueroa and A. Billard, “Discovering hierarchical structure in heterogenous and sequential task demonstrations,” In preparation.

[2] A. L. Pais, K. Umezawa, Y. Nakamura, and A. Billard, “Task parametrization using continuous constraints extracted from human demonstrations,” Accepted, IEEE TRO, 2015.

[3] A. Shukla and A. Billard, “Coupled dynamical system based arm-hand grasping model for learning fast adaptation strategies,” Robotics and Autonomous Systems, vol. 60, no. 3, pp. 424 – 440, 2012.
