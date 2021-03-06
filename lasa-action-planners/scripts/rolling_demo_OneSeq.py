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
import tf
import geometry_msgs.msg
from lasa_perception_module.srv import *

def estimate_action_transforms(phase_name):
    rospy.wait_for_service('attractor_pose')
    try:
      attr_pose = rospy.ServiceProxy('attractor_pose', attractorPose)
      resp1 = attr_pose(phase_name)
      return resp1.dough_found, resp1.object_frame, resp1.reach_center_attractor, resp1.reach_corner_attractor, resp1.roll_attractor, resp1.back_attractor, resp1.area
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e


def PLAN2CTRL_client(phase, object_frame, attractor_frame, timeout):
    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    client = actionlib.SimpleActionClient('plan2ctrl', lasa_action_planners.msg.PLAN2CTRLAction)
    
    
    print "Phase:", phase
    print "Object Frame: ", object_frame
    print "Attractor Frame:", attractor_frame
    print "Timeout: ", timeout
   
    #Waits until the action server has started up and started listening for goals.
    print "waiting for server"
    client.wait_for_server()
    
    #-----------------------------------------------#
    #----- Set of Goals for the Motion Planner -----#
    #-----------------------------------------------#
    
    desired_action = 'LEARNED_MODEL'  
    
    # Do Action from Learned Models    
    if desired_action=='LEARNED_MODEL':
      goal = lasa_action_planners.msg.PLAN2CTRLGoal(action_type=desired_action, action_name = phase, object_frame = object_frame, attractor_frame = attractor_frame, timeout = timeout)
    
    
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
        
        home = geometry_msgs.msg.Transform()
	home.translation.x = -0.45
	home.translation.y = 0.15
	home.translation.z = 0.25
	home.rotation.x = 0.574
	home.rotation.y = 0.819
	home.rotation.z = -0.002
	home.rotation.w = -0.020
	
	fake_object = geometry_msgs.msg.Transform()
	fake_object.translation.x = 0
	fake_object.translation.y = 0
	fake_object.translation.z = 0
	fake_object.rotation.x = 0
	fake_object.rotation.y = 0
	fake_object.rotation.z = 0
	fake_object.rotation.w = 1
	
	print "\n= = = = = Going HOME = = = = = = = = "
        result = PLAN2CTRL_client('back', fake_object, home, 5)
        print "Result:"
        print result.success
        print "\n\n= = = = = = = = = = = = = = = = = = = = = "
        raw_input('Press Enter for Vision')
        print "\n\n= = = = = = = = = = = = = = = = = = = = = "
        
	print "\n\n= = = = = = = = = = = = = = = = = = = = ="
	print "Querying Vision for attractors and object frame"
	print "\n= = = = = = = = = = = = = = = = = = = = ="
	
        dough_found, object_frame, reach_center, reach_corner, roll, back, area = estimate_action_transforms(0)              
        
        if dough_found:
        
	  result = PLAN2CTRL_client('reach', object_frame, reach_corner, 7)
	  print "Result:"		
	  print result.success
	  print "\n\n= = = = = = = = = = = = = = = = = = = = = "
	  raw_input('Press Enter for next phase')
	  print "\n\n= = = = = = = = = = = = = = = = = = = = = "

	  result = PLAN2CTRL_client('roll', object_frame, roll, 10)
	  print "Result:"
	  print result.success
	  print "\n\n= = = = = = = = = = = = = = = = = = = = = "
	  raw_input('Press Enter for next phase')
	  print "\n\n= = = = = = = = = = = = = = = = = = = = = "

	  result = PLAN2CTRL_client('back', object_frame, back, 5)
	  print "Result:"
	  print result.success
	  
	else:
	  print "DOUGH NOT FOUND!!! PLEASE MOVE ROBOT OR PUT DOUGH ON TABLE!!!"


    except rospy.ROSInterruptException:
        print "program interrupted before completion"
        
        
