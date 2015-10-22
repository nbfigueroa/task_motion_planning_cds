#! /usr/bin/env python

# Script for testing PLN2CTRL client 
import roslib; roslib.load_manifest('lasa_action_planners')
import rospy
import numpy
# Import the SimpleActionClient
import actionlib

# Import the messages
import lasa_action_planners.msg
from robohow_common_msgs.msg import MotionPhase
from robohow_common_msgs.msg import MotionModel

def PLAN2CTRL_client():
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('plan2ctrl', lasa_action_planners.msg.PLAN2CTRLAction)
    
    # Waits until the action server has started up and started
    # listening for goals.
    print "waiting for server"
    client.wait_for_server()
    
    #-----------------------------------------------#
    #----- Set of Goals for the Motion Planner -----#
    #-----------------------------------------------#
    
    desired_action = rospy.get_param('desired_action')
    
    if desired_action=='HOME':
      # Go to Home Position	
      goal = lasa_action_planners.msg.PLAN2CTRLGoal(action_type= desired_action)    

    if desired_action=='FIND_TABLE':
      # Find Table (Reach to Dough)
      height = 0.3
      threshold = 5
      goal = lasa_action_planners.msg.PLAN2CTRLGoal(action_type = desired_action, height = height, threshold = threshold)
    
    # Roll Dough Once with constant Force    
    if desired_action=='ROLL_TEST':
      speed = 0.03
      #attractor = numpy.eye(4);
      force = 10;
      goal = lasa_action_planners.msg.PLAN2CTRLGoal(action_type = desired_action, speed = speed, force = force)
    
    # Do Action from Learned Models    
    if desired_action=='LEARNED_MODEL':
      action_name = 'roll'
      goal = lasa_action_planners.msg.PLAN2CTRLGoal(action_type=desired_action, action_name = action_name)
    
    # Sends the goal to the action server.
    print "sending goal", goal
    client.send_goal(goal)
    
    # Waits for the server to finish performing the action.
    print "waiting for result"
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('plan2ctrl_client')
        result = PLAN2CTRL_client()

        print "\n\n= = = = = = = = = = = = = = = = = = = = = "
        print "Result:"
        print result.success

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
        
        
