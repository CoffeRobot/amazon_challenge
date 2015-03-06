import rospy
from my_pr2 import MyPR2
import moveit_commander

pr2 = MyPR2()

# load joint angle dictionary
pr2.load('stack_campbells_on_pringles_dual_arm.pr2')

# initialize
pr2.go_torso(.03)
pr2.go_head('initial')
pr2.go_left_arm('initial')
pr2.go_right_arm('initial')

# forward execution
rospy.sleep(10)
speed = 0.5
pr2.go_right_arm('up', speed)
pr2.go_arms('monitor_close', speed)
pr2.go_left_arm('approach', speed)
pr2.go_left_arm('grasp', speed)
pr2.go_left_gripper(0.06, 30)
rospy.sleep(1)
pr2.go_arms('grasped_up', speed)
pr2.go_right_arm('above_pringles_closer', speed)
pr2.go_left_arm('above_pringles_closer', speed)
pr2.go_left_arm('release', speed)
pr2.go_left_gripper(0.08, 30)
pr2.go_left_arm('post_release0', speed)
pr2.go_left_arm('post_release1', speed)

# run backwards to initial position
speed = 0.5
pr2.go_left_arm('post_release0', speed)
pr2.go_left_arm('release', speed)
pr2.go_left_gripper(0.06, 30)
rospy.sleep(1)
pr2.go_left_arm('above', speed)
pr2.go_left_arm('grasped_up', speed)
pr2.go_left_arm('grasp', speed)
pr2.go_left_gripper(0.08, 30)
pr2.go_left_arm('approach', speed)
pr2.go_left_arm('pre_approach', speed)
pr2.go_left_arm('initial', speed)
pr2.go_right_arm('initial', speed)

# shutdown
pr2.shutdown()
