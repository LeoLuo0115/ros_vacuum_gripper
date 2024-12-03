#!/usr/bin/env python
# -*- coding:utf-8 -*-

from multiprocessing import current_process
from select import select
import sys
import turtle
# sys.path.append('/home/lzw/now_work/ur_ws/src/ur_carry_box')
import rospy
from std_msgs.msg import Float64
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped,Pose
import math
from math import pi,sin,cos
import numpy as np	
import time
from moveit_commander.conversions import pose_to_list
# from pyquaternion import Quaternion
import add_box_in_sence
from std_srvs.srv import Empty

def all_close(goal, actual, tolerance):
	"""
	Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
	@param: goal       A list of floats, a Pose or a PoseStamped
	@param: actual     A list of floats, a Pose or a PoseStamped
	@param: tolerance  A float
	@returns: bool
	"""
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False

	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

class MoveGroupPythonIntefaceTutorial(object):
	"""MoveGroupPythonIntefaceTutorial"""
	def __init__(self):
		super(MoveGroupPythonIntefaceTutorial, self).__init__()
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('move_group_python_interface_tutorial',
						anonymous=True)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group_name = "Arm"
		group = moveit_commander.MoveGroupCommander(group_name)
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
													moveit_msgs.msg.DisplayTrajectory,
													queue_size=20)

		
		planning_frame = group.get_planning_frame()
		print("============ Reference frame: %s" % planning_frame)

		# We can also print the name of the end-effector link for this group:
		eef_link = group.get_end_effector_link()
		print("============ End effector: %s" % eef_link)
		
		arm_links = robot.get_link_names(group=group_name)
		print("============ arm links: %s" % arm_links)

		# We can get a list of all the groups in the robot:
		group_names = robot.get_group_names()
		# print("============ Robot Groups:", robot.get_group_names())

		# Sometimes for debugging it is useful to print the entire state of the
		# robot:
		# print("============ Printing robot state")
		# print(robot.get_current_state())
		## END_SUB_TUTORIAL

		# Misc variables
		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.group = group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names
		self.arm_links = arm_links

	def go_to_joint_state(self, joints):
		group = self.group
		joint_goal = group.get_current_joint_values()
		for i in range(6):
			print("joint "+str(i)+" angle is ", joint_goal[i])
		# joint_goal[0] = 1.57
		# joint_goal[1] = -1.57
		# joint_goal[2] = 1.57
		# joint_goal[3] = 1.57
		# joint_goal[4] = -1.57
		# joint_goal[5] = 0.0
		
		joint_goal = joints
		
		group.go(joint_goal, wait=True)
		group.stop()

		current_joints = self.group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)

	
	def go_to_pose_goal(self, goal):
		'''
		goal : list  [x,y,z, R, P, Y]
		'''
		group = self.group
		current_pose = group.get_current_pose()
		print('current pose is {}'.format(current_pose.pose))
	
		group.set_start_state_to_current_state()
		group.set_pose_target(goal, self.eef_link)
		plan_success, traj, planning_time, error_code = group.plan()
		group.execute(traj)
		group.clear_pose_targets()
		print("============ Move success.")	

def main():
	try:
		tutorial = MoveGroupPythonIntefaceTutorial()
		
		rospy.wait_for_service('/vacuum_gripper/on', 1)
		rospy.wait_for_service('/vacuum_gripper/off', 1)
		_on = rospy.ServiceProxy('/vacuum_gripper/on', Empty)
		_off = rospy.ServiceProxy('/vacuum_gripper/off', Empty)

		put_position = [[1,-0.15],[1.15,0.225],[0.8,-0.225],[0.8,0.225],[0.45,-0.225],[0.45,0.225]]
		for i in range(len(put_position)):
			# get box
			tutorial.go_to_joint_state(joints=[1.57,-1.57,1.57,-1.57,-1.57,0])
			# add box
			add_box_in_sence.spawn_aruco_cube_hover(str(i+1))
			# exit()
			tutorial.go_to_pose_goal([0, 1, 0.37, -pi,0,0])
			# exit()
			time.sleep(1)
			_on()
			time.sleep(2)
			# put box
			tutorial.go_to_pose_goal([0.5, 0.25, 0.65, -pi,0,0])
			# dst pos
			tutorial.go_to_pose_goal([put_position[i][0], put_position[i][1], 0.45, -pi,0,0])
			tutorial.go_to_pose_goal([put_position[i][0], put_position[i][1], 0.20, -pi,0,0])
			time.sleep(1)
			_off()
			time.sleep(1)
			tutorial.go_to_pose_goal([1,0,0.75, -pi,0,0])
		
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
  main()