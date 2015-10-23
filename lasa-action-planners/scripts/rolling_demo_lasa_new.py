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


def send_goal(action_type, action_name, object_frame, attractor_frame, force_gmm_id, timeout):
	#Printing params
	print "Phase:", action_name
    	print "Object Frame: ", object_frame
	print "Attractor Frame:", attractor_frame
        if force_gmm_id!='':	
	    print "Force GMM ID:", force_gmm_id
        print "Timeout: ", timeout

	#Setting Goal for Motion Planner
      	goal = lasa_action_planners.msg.PLAN2CTRLGoal(action_type= action_type, action_name = action_name, object_frame = object_frame, attractor_frame = attractor_frame, force_gmm_id = force_gmm_id, timeout = timeout)

    	# Sends the goal to the action server.
    	print "sending goal", goal
    	client.send_goal(goal)
    
    	# Waits for the server to finish performing the action.
    	print "waiting for result"
    	client.wait_for_result()

    	# Return the result of executing the action
    	return client.get_result()

def SendGoals2MotionPlanner(desired_action):
  
    	print "\n= = = = = Going HOME = = = = = = = = "
        home = geometry_msgs.msg.Transform()
	home.translation.x = -0.517
	home.translation.y = 0.142	
	home.translation.z = 0.188
	home.rotation.x = -0.010
	home.rotation.y = 1.000
	home.rotation.z = -0.016
	home.rotation.w = 0.004
	

	fake_object = geometry_msgs.msg.Transform()
	fake_object.translation.x = 0
	fake_object.translation.y = 0
	fake_object.translation.z = 0
	fake_object.rotation.x = 0
	fake_object.rotation.y = 0
	fake_object.rotation.z = 0
	fake_object.rotation.w = 1

	# Send goal to motion planner
	result = send_goal(desired_action, 'home', fake_object, home, "", 10)
	print "Result:"
	print result.success	

	# STARTING ROLLING SEQUENCE
        dough_found = 0	  
	#desired_dough_area = 0.060 #Approximated Ellipse Area m^2
	desired_dough_area = 0.080 #Approximated Ellipse Area m^2
	area = 0	    
	num_rolls = 0
	max_rolls = 15	

	print "\n\n= = = = = = = = = = = = = = = = = = = = = "
	raw_input('Press Enter to Start New Sequence')
	print "\n\n= = = = = = = = = = = = = = = = = = = = = "

	while area < desired_dough_area and num_rolls<max_rolls:	     	      
	      
	      #Wait a few seconds before starting
	      rospy.sleep(2.)
	      print "\n\n= = = = = = = = = = = = = = = = = = = = ="
	      print "Querying Vision for attractors and object frame"
	      print "\n= = = = = = = = = = = = = = = = = = = = ="	      

	      dough_found, object_frame, reach_center, reach_corner, roll, back, area = estimate_action_transforms(0)
	      
	      print "\n\n= = = = Current Dough AREA: ", area, "= = = = = = = = = "


	      if dough_found:
        					
		#Choose GMM model for force profile depending on the current Dough Area
		if area < 0.025:
			force_gmm_id='first'
		if (area > 0.025 and area < 0.045):
			force_gmm_id='mid'
		if area > 0.045: 
			force_gmm_id='last' 
		 
		# Choose to use Center/Corner Roll attractor
		if num_rolls<10:
			reach=reach_center
			print 'Using CENTER Attractor'
		else:
			reach=reach_corner	
			print 'Using CORNER Attractor'
			
		#reach = reach_corner
		#force_gmm_id='mid'
		
		#result = send_goal(desired_action, 'reach', object_frame, reach, "", 10)
		result = send_goal(desired_action, 'reach', object_frame, reach, "", 10)
		print "\n\n= = = = Starting reaching phase = = = = = "
		print "Result:"		
		print result.success

		result = send_goal(desired_action, 'roll', object_frame, roll, force_gmm_id, 10)
		print "\n\n= = = = Starting rolling phase = = = = = "
		print "Result:"
		print result.success
		

		result = send_goal(desired_action, 'back', object_frame, back, "", 10)
		print "\n\n= = = = Starting back phase = = = = = "
		print "Result:"
		print result.success
		
		num_rolls = num_rolls + 1 
		
		#print "\n\n= = = = = = = = = = = = = = = = = = = = = "
		#raw_input('Press Enter to Start New Sequence')
		#print "\n\n= = = = = = = = = = = = = = = = = = = = = "

	      else:
		print "DOUGH NOT FOUND!!! PLEASE MOVE ROBOT OR PUT DOUGH ON TABLE!!!"
	
	print "Desired area reached: ", area, " with ", num_rolls, " rolls. :)"
    

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('plan2ctrl_client')

    	# Creates the SimpleActionClient, passing the type of the action to the constructor.
    	client = actionlib.SimpleActionClient('plan2ctrl', lasa_action_planners.msg.PLAN2CTRLAction)
    	#Waits until the action server has started up and started listening for goals.
    	print "waiting for server"
    	client.wait_for_server()

	SendGoals2MotionPlanner('LEARNED_MODEL')
	
	       	
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
        
        
