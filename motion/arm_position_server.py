#! /usr/bin/env python

import roslib; roslib.load_manifest('bt_actions')
import rospy
import actionlib
import bt_actions.msg
from my_pr2 import MyPR2
import moveit_commander

class BTAction(object):
  # create messages that are used to publish feedback/result
  _feedback = bt_actions.msg.BTFeedback()
  _result   = bt_actions.msg.BTResult()
  LEFT_ARM = 1
  RIGHT_ARM = 2

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, bt_actions.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    rospy.Subscriber("amazon_next_task", String, get_task)
    self.pr2 = MyPR2()

  def get_task(self, msg):
    text = msg.data
    text = text.replace('[','')
    text = text.replace(']','')
    words = text.split(',')
    self.bin = words[0]
    self.item = words[1]

  def get_arm_to_move(self):
    if self.bin == 'bin_A' or self.bin == 'bin_D' or self.bin == 'bin_G' or self.bin == 'bin_J':  
      return self.LEFT_ARM
    else:
      return self.RIGHT_ARM

    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = True
    
    # publish info to the console for the user
    rospy.loginfo('Starting Action')
    
    # start executing the action
    for i in xrange(1, 5):
      # check that preempt has not been requested by the client
      if self._as.is_preempt_requested():
        #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
        rospy.loginfo('Action Halted')
        self._as.set_preempted()
        success = False
        break

      rospy.loginfo('Executing Action')
      #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES NOT HALT THE ACTION
      speed = 0.5
      arm_to_move = get_arm_to_move()
      query = self.bin + '_init'
      if arm_to_move == self.LEFT_ARM:
        pr2.go_left_arm(query, speed)
      elif: arm_to_move == self.RIGHT_ARM:
        pr2.go_right_arm(query, speed)

      # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep()
      
    if success:
      self.set_status('FAILURE')

    

  def set_status(self,status):
      if status == 'SUCCESS':
        self._feedback.status = 1
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
      elif status == 'FAILURE':
        self._feedback.status = 2
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Failed' % self._action_name)
        self._as.set_succeeded(self._result)
      else:
        rospy.logerr('Action %s: has a wrong return status' % self._action_name)



if __name__ == '__main__':
  rospy.init_node('action')
  BTAction(rospy.get_name())
  rospy.spin()
