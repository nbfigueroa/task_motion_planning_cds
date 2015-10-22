/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 *
 * plan2ctrl_server.cpp
 *
 * Created on : Nov 16, 2014
 * Author     : nbfigueroa
 * Email      : nadia.figueroafernandez@epfl.ch
 * Website    : lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <lasa_action_planners/PLAN2CTRLAction.h>

class PLAN2CTRLAction
{
 protected:

 ros::NodeHandle nh_;
 // NodeHandle instance must be created before this line. Otherwise strange error may occur.
 actionlib::SimpleActionServer<lasa_action_planners::PLAN2CTRLAction> as_; 
 std::string action_name_;
 // create messages that are used to published feedback/result
 lasa_action_planners::PLAN2CTRLFeedback feedback_;
 lasa_action_planners::PLAN2CTRLResult result_;
  
 public:
  
 PLAN2CTRLAction(std::string name) :
 as_(nh_, name, boost::bind(&PLAN2CTRLAction::executeCB, this, _1), false),
 action_name_(name)
 {
   as_.start();
 }
  
 ~PLAN2CTRLAction(void)
 {
 }
  
 void executeCB(const lasa_action_planners::PLAN2CTRLGoalConstPtr &goal)
 {
      // helper variables
      ros::Rate r(1);
      bool success = true;
      
      // initialize progress as null
      feedback_.progress = 0;
      
      
      // start executing the action
      std::string desired_action = goal->action_type;
      std::vector<double> desired_goal_pos = goal->vector1;
      
      std::cout << "Desired Action is " << desired_action << std::endl;
      std::cout << "Desired Goal Position is: " << desired_goal_pos << std::endl;
//       for(int i=0;i<desired_goal_pos.size();i++){
// 	std::cout <<desired_goal_pos[i] << " "; 
//       }
      
      ros::Duration d = ros::Duration(5,0);
      d.sleep();
      
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          // set the action state to preempted
          as_.setPreempted();
          success = false;
      }
      feedback_.progress = 100;
        
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();

  
      if(success)
      {
        result_.success = 100;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
      }
    }
  
  
  };
  
  
  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "plan2ctrl");
  
    PLAN2CTRLAction pizza_making(ros::this_node::getName());
    ros::spin();
  
    return 0;
}