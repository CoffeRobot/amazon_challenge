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
                self._arms_dict = {'left_arm': self._left_arm, 'right_arm': self._right_arm}
                break
            except:
                rospy.sleep(1.0)
                pass


        self._bm = baseMove.baseMove(verbose=False)
        self._bm.setPosTolerance(0.02)
        self._bm.setAngTolerance(0.006)
        self._bm.setLinearGain(1)
        self._bm.setAngularGain(1)

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

        self._widget.arms_start_pos_button.clicked[bool].connect(self._handle_arms_start_pos_button_clicked)

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

    # left arm

    def _handle_l_arm_start_pos_button_clicked(self):
        rospy.loginfo('[GUI]: left arm start pos')
        left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
        joint_pos_goal = left_arm_joint_pos_dict['start']

        self._left_arm.set_joint_value_target(joint_pos_goal)
        self._left_arm.go()

    def _handle_l_arm_row_1_pos_button_clicked(self):
        rospy.loginfo('[GUI]: left arm row 1 pos')
        left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
        joint_pos_goal = left_arm_joint_pos_dict['row_1']

        self._left_arm.set_joint_value_target(joint_pos_goal)
        self._left_arm.go()

    def _handle_l_arm_row_2_pos_button_clicked(self):
        rospy.loginfo('[GUI]: left arm row 2 pos')
        left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
        joint_pos_goal = left_arm_joint_pos_dict['row_2']

        self._left_arm.set_joint_value_target(joint_pos_goal)
        self._left_arm.go()

    def _handle_l_arm_row_3_pos_button_clicked(self):
        rospy.loginfo('[GUI]: left arm row 3 pos')
        left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
        joint_pos_goal = left_arm_joint_pos_dict['row_3']

        self._left_arm.set_joint_value_target(joint_pos_goal)
        self._left_arm.go()

    def _handle_l_arm_row_4_pos_button_clicked(self):
        rospy.loginfo('[GUI]: left arm row 4 pos')
        left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
        joint_pos_goal = left_arm_joint_pos_dict['row_4']

        self._left_arm.set_joint_value_target(joint_pos_goal)
        self._left_arm.go()


    # right arm

    def _handle_r_arm_start_pos_button_clicked(self):
        rospy.loginfo('[GUI]: right arm start pos')
        right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
        joint_pos_goal = right_arm_joint_pos_dict['start']

        self._right_arm.set_joint_value_target(joint_pos_goal)
        self._right_arm.go()

    def _handle_r_arm_row_1_pos_button_clicked(self):
        rospy.loginfo('[GUI]: right arm row 1 pos')
        right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
        joint_pos_goal = right_arm_joint_pos_dict['row_1']

        self._right_arm.set_joint_value_target(joint_pos_goal)
        self._right_arm.go()

    def _handle_r_arm_row_2_pos_button_clicked(self):
        rospy.loginfo('[GUI]: right arm row 2 pos')
        right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
        joint_pos_goal = right_arm_joint_pos_dict['row_2']

        self._right_arm.set_joint_value_target(joint_pos_goal)
        self._right_arm.go()

    def _handle_r_arm_row_3_pos_button_clicked(self):
        rospy.loginfo('[GUI]: right arm row 3 pos')
        right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
        joint_pos_goal = right_arm_joint_pos_dict['row_3']

        self._right_arm.set_joint_value_target(joint_pos_goal)
        self._right_arm.go()

    def _handle_r_arm_row_4_pos_button_clicked(self):
        rospy.loginfo('[GUI]: right arm row 4 pos')
        right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
        joint_pos_goal = right_arm_joint_pos_dict['row_4']

        self._right_arm.set_joint_value_target(joint_pos_goal)
        self._right_arm.go()

    # torso

    def _handle_torso_start_pos_button_clicked(self):
        rospy.loginfo('[GUI]: torso start pos')
        torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
        joint_pos_goal = torso_joint_pos_dict['start']

        self._torso.set_joint_value_target(joint_pos_goal)
        self._torso.go()

    def _handle_torso_row_1_pos_button_clicked(self):
        rospy.loginfo('[GUI]: torso row 1 pos')
        torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
        joint_pos_goal = torso_joint_pos_dict['row_1']

        self._torso.set_joint_value_target(joint_pos_goal)
        self._torso.go()

    def _handle_torso_row_2_pos_button_clicked(self):
        rospy.loginfo('[GUI]: torso row 2 pos')
        torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
        joint_pos_goal = torso_joint_pos_dict['row_2']

        self._torso.set_joint_value_target(joint_pos_goal)
        self._torso.go()

    def _handle_torso_row_3_pos_button_clicked(self):
        rospy.loginfo('[GUI]: torso row 3 pos')
        torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
        joint_pos_goal = torso_joint_pos_dict['row_3']

        self._torso.set_joint_value_target(joint_pos_goal)
        self._torso.go()


    def _handle_torso_row_4_pos_button_clicked(self):
        rospy.loginfo('[GUI]: torso row 4 pos')
        torso_joint_pos_dict = rospy.get_param('/torso_joint_pos_dict')
        joint_pos_goal = torso_joint_pos_dict['row_4']

        self._torso.set_joint_value_target(joint_pos_goal)
        self._torso.go()

    def _handle_arms_start_pos_button_clicked(self):
        rospy.loginfo('[GUI]: arms start pos')
        left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
        right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
        left_arm_joint_pos_goal = left_arm_joint_pos_dict['start']
        right_arm_joint_pos_goal = right_arm_joint_pos_dict['start']
        joint_pos_goal = left_arm_joint_pos_goal + right_arm_joint_pos_goal

        self._arms.set_joint_value_target(joint_pos_goal)
        self._arms.go()

    def _handle_base_col_1_pos_button_clicked(self):
        # put arms in start position
        self._handle_arms_start_pos_button_clicked()
        rospy.loginfo('[GUI]: base col 1 pos')
        base_pos_dict = rospy.get_param('/base_pos_dict')

        base_pos_goal = base_pos_dict['column_1']
        self._bm.goAngle(base_pos_goal[5])
        self._bm.goPosition(base_pos_goal[0:2])
        self._bm.goAngle(base_pos_goal[5])

    def _handle_base_col_2_pos_button_clicked(self):
        # put arms in start position
        self._handle_arms_start_pos_button_clicked()
        rospy.loginfo('[GUI]: base col 2 pos')
        base_pos_dict = rospy.get_param('/base_pos_dict')

        base_pos_goal = base_pos_dict['column_2']
        self._bm.goAngle(base_pos_goal[5])
        self._bm.goPosition(base_pos_goal[0:2])
        self._bm.goAngle(base_pos_goal[5])

    def _handle_base_col_3_pos_button_clicked(self):
        # put arms in start position
        self._handle_arms_start_pos_button_clicked()
        rospy.loginfo('[GUI]: base col 3 pos')
        base_pos_dict = rospy.get_param('/base_pos_dict')

        base_pos_goal = base_pos_dict['column_3']
        self._bm.goAngle(base_pos_goal[5])
        self._bm.goPosition(base_pos_goal[0:2])
        self._bm.goAngle(base_pos_goal[5])

