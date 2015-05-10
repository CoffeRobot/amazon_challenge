#!/usr/bin/env python

#   pr2_amazon_challenge_gui
#
#   Created on: April 29, 2015
#   Authors:   Francisco Vina
#             fevb <at> kth.se
#

#  Copyright (c) 2015, Francisco Vina, CVAP, KTH
#    All rights reserved.

#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions are met:
#       * Redistributions of source code must retain the above copyright
#         notice, this list of conditions and the following disclaimer.
#       * Redistributions in binary form must reproduce the above copyright
#         notice, this list of conditions and the following disclaimer in the
#         documentation and/or other materials provided with the distribution.
#       * Neither the name of KTH nor the
#         names of its contributors may be used to endorse or promote products
#         derived from this software without specific prior written permission.

#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#    DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
#    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QMainWindow, QGraphicsView
import numpy as np
import moveit_commander
import copy
import random

from std_srvs.srv import Empty

from pr2_controllers_msgs.msg import Pr2GripperCommand

from calibrateBase import baseMove

class PR2AmazonChallengePlugin(Plugin):

    def __init__(self, context):
        super(PR2AmazonChallengePlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('PR2AmazonChallengePlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QMainWindow()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_amazon_challenge'), 'resource', 'pr2amazonchallenge.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('PR2AmazonChallengePlugin')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        while not rospy.is_shutdown():
            rospy.loginfo('[GUI]: connecting to moveit...')
            try:
                self._left_arm = moveit_commander.MoveGroupCommander('left_arm')
                self._right_arm = moveit_commander.MoveGroupCommander('right_arm')
                self._arms = moveit_commander.MoveGroupCommander('arms')
                self._torso = moveit_commander.MoveGroupCommander('torso')
                self._head = moveit_commander.MoveGroupCommander('head')
                self._arms_dict = {'left_arm': self._left_arm, 'right_arm': self._right_arm}
                break
            except:
                rospy.sleep(1.0)
                pass


        while not rospy.is_shutdown():
            try:
                base_move_params = rospy.get_param('/base_move')
                self._tool_size = rospy.get_param('/tool_size', [0.16, 0.02, 0.04])
                self._contest = rospy.get_param('/contest', True)
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        # get base_move parameters

        self._bm = baseMove.baseMove(verbose=False)
        self._bm.setPosTolerance(base_move_params['pos_tolerance'])
        self._bm.setAngTolerance(base_move_params['ang_tolerance'])
        self._bm.setLinearGain(base_move_params['linear_gain'])
        self._bm.setAngularGain(base_move_params['angular_gain'])

        self._l_gripper_pub = rospy.Publisher('/l_gripper_controller/command', Pr2GripperCommand)


        self._item = ''

        if self._contest:
            self._length_tool = 0.18 + self._tool_size[0]
        else:
            self._length_tool = 0.216 + self._tool_size[0]

        # button click callbacks
        self._widget.l_arm_start_pos_button.clicked[bool].connect(self._handle_l_arm_start_pos_button_clicked)
        self._widget.l_arm_row_1_pos_button.clicked[bool].connect(self._handle_l_arm_row_1_pos_button_clicked)
        self._widget.l_arm_row_2_pos_button.clicked[bool].connect(self._handle_l_arm_row_2_pos_button_clicked)
        self._widget.l_arm_row_3_pos_button.clicked[bool].connect(self._handle_l_arm_row_3_pos_button_clicked)
        self._widget.l_arm_row_4_pos_button.clicked[bool].connect(self._handle_l_arm_row_4_pos_button_clicked)

        self._widget.r_arm_start_pos_button.clicked[bool].connect(self._handle_r_arm_start_pos_button_clicked)
        self._widget.r_arm_row_1_pos_button.clicked[bool].connect(self._handle_r_arm_row_1_pos_button_clicked)
        self._widget.r_arm_row_2_pos_button.clicked[bool].connect(self._handle_r_arm_row_2_pos_button_clicked)
        self._widget.r_arm_row_3_pos_button.clicked[bool].connect(self._handle_r_arm_row_3_pos_button_clicked)
        self._widget.r_arm_row_4_pos_button.clicked[bool].connect(self._handle_r_arm_row_4_pos_button_clicked)

        self._widget.torso_start_pos_button.clicked[bool].connect(self._handle_torso_start_pos_button_clicked)
        self._widget.torso_row_1_pos_button.clicked[bool].connect(self._handle_torso_row_1_pos_button_clicked)
        self._widget.torso_row_2_pos_button.clicked[bool].connect(self._handle_torso_row_2_pos_button_clicked)
        self._widget.torso_row_3_pos_button.clicked[bool].connect(self._handle_torso_row_3_pos_button_clicked)
        self._widget.torso_row_4_pos_button.clicked[bool].connect(self._handle_torso_row_4_pos_button_clicked)


        self._widget.base_calibration_button.clicked[bool].connect(self._handle_base_col_1_pos_button_clicked)
        self._widget.base_col_1_pos_button.clicked[bool].connect(self._handle_base_col_1_pos_button_clicked)
        self._widget.base_col_2_pos_button.clicked[bool].connect(self._handle_base_col_2_pos_button_clicked)
        self._widget.base_col_3_pos_button.clicked[bool].connect(self._handle_base_col_3_pos_button_clicked)

        self._widget.head_start_pos_button.clicked[bool].connect(self._handle_head_row_1_pos_button_clicked)
        self._widget.head_row_1_pos_button.clicked[bool].connect(self._handle_head_row_1_pos_button_clicked)
        self._widget.head_row_2_pos_button.clicked[bool].connect(self._handle_head_row_2_pos_button_clicked)
        self._widget.head_row_3_pos_button.clicked[bool].connect(self._handle_head_row_3_pos_button_clicked)
        self._widget.head_row_4_pos_button.clicked[bool].connect(self._handle_head_row_4_pos_button_clicked)


        self._widget.arms_start_pos_button.clicked[bool].connect(self._handle_arms_start_pos_button_clicked)

        self._widget.drop_button.clicked[bool].connect(self._handle_drop_button_clicked)
        self._widget.drop_cheezit_button.clicked[bool].connect(self._handle_drop_cheezit_button_clicked)
        self._widget.drop_oreo_button.clicked[bool].connect(self._handle_drop_oreo_button_clicked)


        self._widget.pc_perception_button.clicked[bool].connect(self._handle_pc_perception_button_clicked)
        self._widget.detector_button.clicked[bool].connect(self._handle_detector_button_clicked)
        self._widget.pregrasp_button.clicked[bool].connect(self._handle_pregrasp_button_clicked)

        self._widget.start_bt_button.clicked[bool].connect(self._handle_start_bt_button_clicked)

        self._mode = 'pregrasp'

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_amazon_challenge plugin')
        # group.add_argument('bagfiles', type=argparse.FileType('r'), nargs='*', default=[], help='Bagfiles to load')


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


    # mode callback
    def _handle_pc_perception_button_clicked(self):
        self._mode = 'point_cloud_perception'
        rospy.loginfo('[GUI]: point_cloud_perception mode')

    def _handle_detector_button_clicked(self):
        self._mode = 'detector'
        rospy.loginfo('[GUI]: detector mode')


    def _handle_pregrasp_button_clicked(self):
        self._mode = 'pregrasp'
        rospy.loginfo('[GUI]: pregrasp mode')


    # left arm

    def _handle_l_arm_start_pos_button_clicked(self):
        rospy.loginfo('[GUI]: left arm start pos')

        while not rospy.is_shutdown():
            try:
                left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = left_arm_joint_pos_dict['start']

        self._left_arm.set_joint_value_target(joint_pos_goal)
        self._left_arm.go()

    def _handle_l_arm_row_1_pos_button_clicked(self):
        rospy.loginfo('[GUI]: left arm row 1 pos')

        while not rospy.is_shutdown():
            try:
                left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = left_arm_joint_pos_dict[self._mode]['row_1']

        self._left_arm.set_joint_value_target(joint_pos_goal)
        self._left_arm.go()

    def _handle_l_arm_row_2_pos_button_clicked(self):
        rospy.loginfo('[GUI]: left arm row 2 pos')

        while not rospy.is_shutdown():
            try:
                left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = left_arm_joint_pos_dict[self._mode]['row_2']

        self._left_arm.set_joint_value_target(joint_pos_goal)
        self._left_arm.go()

    def _handle_l_arm_row_3_pos_button_clicked(self):
        rospy.loginfo('[GUI]: left arm row 3 pos')


        while not rospy.is_shutdown():
            try:
                left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = left_arm_joint_pos_dict[self._mode]['row_3']

        self._left_arm.set_joint_value_target(joint_pos_goal)
        self._left_arm.go()

    def _handle_l_arm_row_4_pos_button_clicked(self):
        rospy.loginfo('[GUI]: left arm row 4 pos')

        while not rospy.is_shutdown():
            try:
                left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = left_arm_joint_pos_dict[self._mode]['row_4']

        self._left_arm.set_joint_value_target(joint_pos_goal)
        self._left_arm.go()


    # right arm

    def _handle_r_arm_start_pos_button_clicked(self):
        rospy.loginfo('[GUI]: right arm start pos')

        while not rospy.is_shutdown():
            try:
                right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = right_arm_joint_pos_dict['start']

        self._right_arm.set_joint_value_target(joint_pos_goal)
        self._right_arm.go()

    def _handle_r_arm_row_1_pos_button_clicked(self):
        rospy.loginfo('[GUI]: right arm row 1 pos')

        while not rospy.is_shutdown():
            try:
                right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = right_arm_joint_pos_dict[self._mode]['row_1']

        self._right_arm.set_joint_value_target(joint_pos_goal)
        self._right_arm.go()

    def _handle_r_arm_row_2_pos_button_clicked(self):
        rospy.loginfo('[GUI]: right arm row 2 pos')

        while not rospy.is_shutdown():
            try:
                right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = right_arm_joint_pos_dict[self._mode]['row_2']

        self._right_arm.set_joint_value_target(joint_pos_goal)
        self._right_arm.go()

    def _handle_r_arm_row_3_pos_button_clicked(self):
        rospy.loginfo('[GUI]: right arm row 3 pos')

        while not rospy.is_shutdown():
            try:
                right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = right_arm_joint_pos_dict[self._mode]['row_3']

        self._right_arm.set_joint_value_target(joint_pos_goal)
        self._right_arm.go()

    def _handle_r_arm_row_4_pos_button_clicked(self):
        rospy.loginfo('[GUI]: right arm row 4 pos')

        while not rospy.is_shutdown():
            try:
                right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = right_arm_joint_pos_dict[self._mode]['row_4']

        self._right_arm.set_joint_value_target(joint_pos_goal)
        self._right_arm.go()

    # torso

    def _handle_torso_start_pos_button_clicked(self):
        rospy.loginfo('[GUI]: torso start pos')

        while not rospy.is_shutdown():
            try:
                torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = torso_joint_pos_dict['start']

        self._torso.set_joint_value_target(joint_pos_goal)
        self._torso.go()

    def _handle_torso_row_1_pos_button_clicked(self):
        rospy.loginfo('[GUI]: torso row 1 pos')

        while not rospy.is_shutdown():
            try:
                torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = torso_joint_pos_dict[self._mode]['row_1']

        self._torso.set_joint_value_target(joint_pos_goal)
        self._torso.go()

    def _handle_torso_row_2_pos_button_clicked(self):
        rospy.loginfo('[GUI]: torso row 2 pos')

        while not rospy.is_shutdown():
            try:
                torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = torso_joint_pos_dict[self._mode]['row_2']

        self._torso.set_joint_value_target(joint_pos_goal)
        self._torso.go()

    def _handle_torso_row_3_pos_button_clicked(self):
        rospy.loginfo('[GUI]: torso row 3 pos')

        while not rospy.is_shutdown():
            try:
                torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = torso_joint_pos_dict[self._mode]['row_3']

        self._torso.set_joint_value_target(joint_pos_goal)
        self._torso.go()


    def _handle_torso_row_4_pos_button_clicked(self):
        rospy.loginfo('[GUI]: torso row 4 pos')

        while not rospy.is_shutdown():
            try:
                torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        joint_pos_goal = torso_joint_pos_dict[self._mode]['row_4']

        self._torso.set_joint_value_target(joint_pos_goal)
        self._torso.go()

    def _handle_arms_start_pos_button_clicked(self):
        rospy.loginfo('[GUI]: arms start pos')

        while not rospy.is_shutdown():
            try:
                left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
                right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        left_arm_joint_pos_goal = left_arm_joint_pos_dict['start']
        right_arm_joint_pos_goal = right_arm_joint_pos_dict['start']
        joint_pos_goal = left_arm_joint_pos_goal + right_arm_joint_pos_goal

        self._arms.set_joint_value_target(joint_pos_goal)
        self._arms.go()

    def _handle_base_col_1_pos_button_clicked(self):
        # put arms in start position
        self._handle_arms_start_pos_button_clicked()
        rospy.loginfo('[GUI]: base col 1 pos')

        while not rospy.is_shutdown():
            try:
                base_pos_dict = rospy.get_param('/base_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        base_pos_goal = base_pos_dict['column_1']

        rospy.loginfo('Base setpoint: ')
        rospy.loginfo(base_pos_goal)

        self._bm.goAngle(base_pos_goal[5])
        self._bm.goPosition(base_pos_goal[0:2])
        self._bm.goAngle(base_pos_goal[5])

    def _handle_base_col_2_pos_button_clicked(self):
        # put arms in start position
        self._handle_arms_start_pos_button_clicked()
        rospy.loginfo('[GUI]: base col 2 pos')

        while not rospy.is_shutdown():
            try:
                base_pos_dict = rospy.get_param('/base_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        base_pos_goal = base_pos_dict['column_2']

        rospy.loginfo('Base setpoint: ')
        rospy.loginfo(base_pos_goal)

        self._bm.goAngle(base_pos_goal[5])
        self._bm.goPosition(base_pos_goal[0:2])
        self._bm.goAngle(base_pos_goal[5])

    def _handle_base_col_3_pos_button_clicked(self):
        # put arms in start position
        self._handle_arms_start_pos_button_clicked()
        rospy.loginfo('[GUI]: base col 3 pos')

        while not rospy.is_shutdown():
            try:
                base_pos_dict = rospy.get_param('/base_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        base_pos_goal = base_pos_dict['column_3']

        rospy.loginfo('Base setpoint: ')
        rospy.loginfo(base_pos_goal)

        self._bm.goAngle(base_pos_goal[5])
        self._bm.goPosition(base_pos_goal[0:2])
        self._bm.goAngle(base_pos_goal[5])

    def _handle_head_row_1_pos_button_clicked(self):
        rospy.loginfo('[GUI]: head row 1 pos')

        while not rospy.is_shutdown():
            try:
                head_joint_pos_dict = rospy.get_param('/head_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        head_pos_goal = head_joint_pos_dict['row_1']

        self._head.set_joint_value_target(head_pos_goal)
        self._head.go()

    def _handle_head_row_2_pos_button_clicked(self):
        rospy.loginfo('[GUI]: head row 2 pos')

        while not rospy.is_shutdown():
            try:
                head_joint_pos_dict = rospy.get_param('/head_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        head_pos_goal = head_joint_pos_dict['row_2']

        self._head.set_joint_value_target(head_pos_goal)
        self._head.go()

    def _handle_head_row_3_pos_button_clicked(self):
        rospy.loginfo('[GUI]: head row 3 pos')

        while not rospy.is_shutdown():
            try:
                head_joint_pos_dict = rospy.get_param('/head_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        head_pos_goal = head_joint_pos_dict['row_3']

        self._head.set_joint_value_target(head_pos_goal)
        self._head.go()

    def _handle_head_row_4_pos_button_clicked(self):
        rospy.loginfo('[GUI]: head row 4 pos')

        while not rospy.is_shutdown():
            try:
                head_joint_pos_dict = rospy.get_param('/head_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        head_pos_goal = head_joint_pos_dict['row_4']

        self._head.set_joint_value_target(head_pos_goal)
        self._head.go()


    def _handle_drop_button_clicked(self):
        rospy.loginfo('[GUI]: going to drop object')

        while not rospy.is_shutdown():
            try:
                base_pos_dict = rospy.get_param('/base_pos_dict')
                torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
                left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
                dropping_height = rospy.get_param('/dropping_height', 0.255)
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue



        # move to the right
        base_pos_goal = base_pos_dict['drop']['move_right']
        self._bm.goAngle(base_pos_goal[5])
        self._bm.goPosition(base_pos_goal[0:2])
        self._bm.goAngle(base_pos_goal[5])

        # retreat
        base_pos_goal = base_pos_dict['drop']['retreat'+self._item]
        self._bm.goAngle(base_pos_goal[5])
        self._bm.goPosition(base_pos_goal[0:2])
        self._bm.goAngle(base_pos_goal[5])

        # torso to drop position
        joint_pos_goal = torso_joint_pos_dict['drop']

        self._torso.set_joint_value_target(joint_pos_goal)
        self._torso.go()

        # move left arm to drop position
        joint_pos_goal = left_arm_joint_pos_dict['drop']

        self._left_arm.set_joint_value_target(joint_pos_goal)
        self._left_arm.go()

        # move base to the left towards the bin
        move_base_y = base_pos_dict['drop']['move_left_y']
        base_pos_goal[1] = move_base_y

        rospy.loginfo('[dropping_server]: moving base left towards the bin')
        self._bm.goAngle(base_pos_goal[5])
        self._bm.goPosition(base_pos_goal[0:2])
        self._bm.goAngle(base_pos_goal[5])

        ######################################
        # move left arm down
        # calculate how much to go down
        z_init = copy.deepcopy(self._left_arm.get_current_pose().pose.position.z)

        z_desired = dropping_height # maximum 30 cm dropping height
        waypoints = []

        #waypoints.append(self._left_arm.get_current_pose().pose)

        wpose = copy.deepcopy(self._left_arm.get_current_pose().pose)
        wpose.position.z = z_desired + self._length_tool

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self._left_arm.compute_cartesian_path(waypoints, 0.005, 0.0)

        self._left_arm.execute(plan)

        ######################################
        # release gripper
        gripper_command_msg = Pr2GripperCommand()
        gripper_command_msg.max_effort = 40.0
        gripper_command_msg.position = 10.0

        self._l_gripper_pub.publish(gripper_command_msg)
        t_init = rospy.Time.now()

        r = rospy.Rate(1.0)

        while (rospy.Time.now()-t_init).to_sec() < 5.0 and not rospy.is_shutdown():
            r.sleep()

        ######################################
        # move left arm up
        # calculate how much to go up

        z_desired = z_init # maximum 30 cm dropping height
        waypoints = []

        #waypoints.append(self._left_arm.get_current_pose().pose)

        wpose = copy.deepcopy(self._left_arm.get_current_pose().pose)
        wpose.position.z = z_desired + self._length_tool

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self._left_arm.compute_cartesian_path(waypoints, 0.005, 0.0)

        self._left_arm.execute(plan)
        self._item = ''


    def _handle_drop_cheezit_button_clicked(self):
        self._item = '_cheezit_big_original'
        self._handle_drop_button_clicked()

    def _handle_drop_oreo_button_clicked(self):
        self._item = '_oreo_mega_stuf'
        self._handle_drop_button_clicked()


    def _handle_start_bt_button_clicked(self):
        rospy.loginfo('[GUI]: start bt button clicked')
        rospy.loginfo('[GUI]: waiting for /kick_ass service')
        rospy.wait_for_service('/kick_ass')
        srv = rospy.ServiceProxy('/kick_ass', Empty)

        rospy.loginfo('[GUI]: BT started!!!')

        srv.call()


