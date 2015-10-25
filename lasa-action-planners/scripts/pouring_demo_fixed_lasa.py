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

	# Pouring Object (i.e. dough/plate/etc)
	#pouring_object = geometry_msgs.msg.Transform()
	#pouring_object.translation.x = -0.549
	#pouring_object.translation.y = -0.328
	#pouring_object.translation.z = -0.044
	#pouring_object.rotation.x = 0
	#pouring_object.rotation.y = 0
	#pouring_object.rotation.z = 0.999
	#pouring_object.rotation.w = 0.037

	#Dough Frame in world
	pouring_object = geometry_msgs.msg.Transform()
#	pouring_object.translation.x = -0.40600
	pouring_object.translation.x = -0.60600
	pouring_object.translation.y = -0.22
	pouring_object.translation.z = -0.044
	pouring_object.rotation.x = 0
	pouring_object.rotation.y = 0
	pouring_object.rotation.z = 0.994362
	pouring_object.rotation.w = 0.106039

	#ROBOT RF
	fake_object = geometry_msgs.msg.Transform()
	fake_object.translation.x = 0
	fake_object.translation.y = 0
	fake_object.translation.z = 0
	fake_object.rotation.x = 0
	fake_object.rotation.y = 0
	fake_object.rotation.z = 0
	fake_object.rotation.w = 1

	# Good Starting configuration for pouring on LWR LASA
        home = geometry_msgs.msg.Transform()
	#home.translation.x = -0.466
	#home.translation.y = -0.0318
	#home.translation.z = 0.3541
	#home.rotation.w = 0.7927
	#home.rotation.x = 0.001387
	#home.rotation.y = -0.05438
	#home.rotation.z = -0.6069

	home.translation.x = -0.483
	home.translation.y = 0.091
	home.translation.z = 0.361
	home.rotation.w =  0.701
	home.rotation.x = -0.257
	home.rotation.y = -0.227
	home.rotation.z = -0.625

	# Reaching Phase Attractor in Dough RF	
	reach_attr = geometry_msgs.msg.Transform()
	#reach_attr.translation.x = -0.0326
	#reach_attr.translation.y = -0.1367
	#reach_attr.translation.z = 0.3363
	#reach_attr.rotation.w = 0.705532
	#reach_attr.rotation.x = -0.0329184
	#reach_attr.rotation.y = 0.00219692
	#reach_attr.rotation.z = 0.606422

	# Reaching Phase Attractor in ROBOT RF
	#- Translation: [-0.482, -0.086, 0.289]
	#- Rotation: in Quaternion [-0.085, -0.078, -0.662, 0.741]
	reach_attr.translation.x = -0.482
	reach_attr.translation.y = -0.086
	reach_attr.translation.z = 0.289
	reach_attr.rotation.w = 0.741
	reach_attr.rotation.x = -0.085
	reach_attr.rotation.y = -0.078
	reach_attr.rotation.z = -0.662


	# Pouring Phase Attractor in Dough RF	
	pour_attr = geometry_msgs.msg.Transform()
	#pour_attr.translation.x =-0.0326 
	#pour_attr.translation.y = -0.1367
	#pour_attr.translation.z = 0.2863
	#pour_attr.rotation.x = -0.117367151246992
	#pour_attr.rotation.y = 0.0454380180012176
	#pour_attr.rotation.z = 0.880185368094969
	#pour_attr.rotation.w = 0.457639657531724


	# Pouring Phase Attractor in ROBOT RF
	#- Translation: [-0.478, -0.184, 0.248]
	#- Rotation: in Quaternion [0.133, 0.112, -0.673, 0.719]
	pour_attr.translation.x = -0.478
	pour_attr.translation.y = -0.184
	pour_attr.translation.z = 0.248
	pour_attr.rotation.x = 0.133
	pour_attr.rotation.y =  0.112
	pour_attr.rotation.z = -0.673
	pour_attr.rotation.w = 0.719

	# Back Phase Attractor
	back_attr = geometry_msgs.msg.Transform()
	back_attr.translation.x = -0.0326
	back_attr.translation.y = -0.1367	
	back_attr.translation.z = 0.3063
	back_attr.rotation.w = 0.705532
	back_attr.rotation.x = -0.0329184
	back_attr.rotation.y = 0.00219692
	back_attr.rotation.z = 0.606422

	print "\n= = = = = Going to a HOME position= = = = = = = = "
        result = PLAN2CTRL_client('home', fake_object, home, 10)
        print "Result:"
        print result.success
        
        print "\n\n= = = = = = = = = = = = = = = = = = = = = "
	raw_input('Press Enter to start pouring')
	print "\n\n= = = = = = = = = = = = = = = = = = = = = "
	
	#result = PLAN2CTRL_client('reach', fake_object, reach_attr, 10)
	print "Result:"		
	#print result.success

	result = PLAN2CTRL_client('pour', fake_object, pour_attr, 10)
	print "Result:"		
	print result.success
	
	#Wait a few seconds before going back
	rospy.sleep(2.)

	result = PLAN2CTRL_client('back',  fake_object, home, 10)
	print "Result:"
	print result.success

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
        
        
