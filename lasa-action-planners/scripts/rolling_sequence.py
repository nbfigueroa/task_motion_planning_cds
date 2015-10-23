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
from lasa_perception_module.srv import *

def estimate_action_transforms(phase_name):
    rospy.wait_for_service('attractor_pose')
    try:
      attr_pose = rospy.ServiceProxy('attractor_pose', attractorPose)
      resp1 = attr_pose(phase_name)
      return resp1.object_frame, resp1.attractor
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e


def PLAN2CTRL_client(phase):
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('plan2ctrl', lasa_action_planners.msg.PLAN2CTRLAction)
    
    # Waits until the action server has started up and started
    # listening for goals.
    print "waiting for server"
    client.wait_for_server()
    
    #-----------------------------------------------#
    #----- Set of Goals for the Motion Planner -----#
    #-----------------------------------------------#
    
    desired_action = 'LEARNED_MODEL'    
    
    # Do Action from Learned Models    
    if desired_action=='LEARNED_MODEL':
      action_name = phase
      object_frame, attractor = estimate_action_transforms(action_name)
      goal = lasa_action_planners.msg.PLAN2CTRLGoal(action_type=desired_action, action_name = action_name, object_frame = object_frame, attractor = attractor)
    
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

        result = PLAN2CTRL_client('reach')
        print "Result:"
        print result.success
        print "\n\n= = = = = = = = = = = = = = = = = = = = = "
        raw_input('Press Enter for next phase')
        print "\n\n= = = = = = = = = = = = = = = = = = = = = "

        result = PLAN2CTRL_client('roll')
        print "Result:"
        print result.success
        print "\n\n= = = = = = = = = = = = = = = = = = = = = "
        raw_input('Press Enter for next phase')
        print "\n\n= = = = = = = = = = = = = = = = = = = = = "

        result = PLAN2CTRL_client('back')
        print "Result:"
        print result.success


    except rospy.ROSInterruptException:
        print "program interrupted before completion"
        
        
