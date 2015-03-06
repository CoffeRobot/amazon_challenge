import sys
sys.path.append('/opt/ros/groovy/stacks/pr2_controllers/pr2_controllers_msgs/src')
import pickle
import rospy
import copy
import moveit_commander
from pr2_controllers_msgs.msg import Pr2GripperCommandActionGoal
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose


def scale_trajectory_speed(traj, scale):
    # Create a new trajectory object
    new_traj = RobotTrajectory()
    # Initialize the new trajectory to be the same as the planned trajectory
    new_traj.joint_trajectory = traj.joint_trajectory
    # Get the number of joints involved
    n_joints = len(traj.joint_trajectory.joint_names)
    # Get the number of points on the trajectory
    n_points = len(traj.joint_trajectory.points)
    # Store the trajectory points
    points = list(traj.joint_trajectory.points)
    # Cycle through all points and scale the time from start, speed and acceleration
    for i in range(n_points):
        point = JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions
        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale
        points[i] = point
    # Assign the modified points to the new trajectory
    new_traj.joint_trajectory.points = points
    # Return the new trajectory
    return new_traj


class MyPR2:
    """
    Convenience class to record and execute PR2 joint-angle and gripper configurations
    Allows allows loading and saving a set of pre-programmed configurations as a dictionary
    """

    def __init__(self):
        """Initialize moveit! and bring up a node to control the grippers
        """
        rospy.init_node('my_pr2_control_node', anonymous=True)
        self.l_gripper_pub = rospy.Publisher('/l_gripper_controller/gripper_action/goal', Pr2GripperCommandActionGoal)
        self.r_gripper_pub = rospy.Publisher('/r_gripper_controller/gripper_action/goal', Pr2GripperCommandActionGoal)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.torso = moveit_commander.MoveGroupCommander('torso')
        self.head = moveit_commander.MoveGroupCommander('head')
        self.arms = moveit_commander.MoveGroupCommander('arms')
        self.left_arm = moveit_commander.MoveGroupCommander('left_arm')
        self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
        self.head_dict = {}
        self.arms_dict = {}
        self.left_arm_dict = {}
        self.right_arm_dict = {}

    def save(self, file_name):
        """Save the current joint angle dictionary
        """
        with open(file_name, 'wb') as handle:
            pickle.dump(self.head_dict, handle)
            pickle.dump(self.arms_dict, handle)
            pickle.dump(self.left_arm_dict, handle)
            pickle.dump(self.right_arm_dict, handle)

    def load(self, file_name):
        """Load a joint angle dictionary
        """
        with open(file_name, 'rb') as handle:
            self.head_dict = pickle.load(handle)
            self.arms_dict = pickle.load(handle)
            self.left_arm_dict = pickle.load(handle)
            self.right_arm_dict = pickle.load(handle)

    def store_head(self, name):
        """Store current head position
        """
        self.head_dict[name] = self.head.get_current_joint_values()

    def store_arms(self, name):
        """Store current arms position
        """
        self.arms_dict[name] = self.arms.get_current_joint_values()

    def store_left_arm(self, name):
        """Store current left arm position
        """
        self.left_arm_dict[name] = self.left_arm.get_current_joint_values()

    def store_right_arm(self, name):
        """Store current right arm position
        """
        self.right_arm_dict[name] = self.right_arm.get_current_joint_values()

    def go_torso(self, position):
        """Move torso to position at max speed
        """
        self.torso.set_joint_value_target([position])
        self.torso.go()

    def go_left_gripper(self, position, max_effort):
        """Move left gripper to position with max_effort
        """
        ope = Pr2GripperCommandActionGoal()
        ope.goal.command.position = position
        ope.goal.command.max_effort = max_effort
        self.l_gripper_pub.publish(ope)

    def go_right_gripper(self, position, max_effort):
        """Move right gripper to position with max_effort
        """
        ope = Pr2GripperCommandActionGoal()
        ope.goal.command.position = position
        ope.goal.command.max_effort = max_effort
        self.r_gripper_pub.publish(ope)

    def go_head(self, name, speed=1.0):
        """Move head to named joint state at speed relative to max speed
        """
        self.move_group(self.head, self.head_dict[name], speed)

    def go_arms(self, name, speed=1.0):
        """Move arms to named joint state at speed relative to max speed
        """
        self.move_group(self.arms, self.arms_dict[name], speed)

    def go_left_arm(self, name, speed=1.0):
        """Move left arm to named joint state at speed relative to max speed
        """
        self.move_group(self.left_arm, self.left_arm_dict[name], speed)

    def go_right_arm(self, name, speed=1.0):
        """Move right arm to named joint state at speed relative to max speed
        """
        self.move_group(self.right_arm, self.right_arm_dict[name], speed)

    @staticmethod
    def move_group(group, target, speed):
        # try:
        group.set_joint_value_target(target)
        # except Exception:
        #     pass
        trajectory = group.plan()
        if speed < 1.0:
            scaled_trajectory = scale_trajectory_speed(trajectory, speed)
        else:
            scaled_trajectory = trajectory
        group.execute(scaled_trajectory)

    @staticmethod
    def shutdown():
        """Shutdown moveit!
        """
        moveit_commander.roscpp_shutdown()
