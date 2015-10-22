#! /usr/bin/env python

import roslib; roslib.load_manifest('lasa_action_planners')
import rospy

import rospkg
import actionlib
import math
import numpy

from std_msgs.msg import String
#from robohow_common_msgs.msg import MotionPhase
#from robohow_common_msgs.msg import MotionModel
#from robohow_common_msgs.msg import GaussianMixtureModel
#from robohow_common_msgs.msg import GaussianDistribution
import lasa_action_planners.msg

class PLAN2CTRLAction(object):
  # create messages that are used to publish feedback/result
  _feedback = lasa_action_planners.msg.PLAN2CTRLFeedback()
  _result   = lasa_action_planners.msg.PLAN2CTRLResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, lasa_action_planners.msg.PLAN2CTRLAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(10)
    success = True
    
    # initialize progress as null
    self._feedback.progress = 0
    
    # publish info to the console for the user
    #rospy.loginfo('Starting loading of parameters to parameter server')
    
    desired_action = goal.action_type
    desired_goal_pos = goal.vector1
    print "Desired action: ",desired_action
    print "Desired Goal Pos: ", desired_goal_pos
    
    self._feedback.progress = 100      
    
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
      rospy.loginfo('%s: Preempted' % self._action_name)
      self._feedback.progress = 0
      self._as.set_preempted()
      success = False

    # publish the feedback
    self._as.publish_feedback(self._feedback)
    r.sleep()
    
    rospy.sleep(5)
    
    if success:
      self._result.success = 100
      self._feedback.progress = 100
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._as.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('plan2ctrl')
  PLAN2CTRLAction(rospy.get_name())
  rospy.spin()
