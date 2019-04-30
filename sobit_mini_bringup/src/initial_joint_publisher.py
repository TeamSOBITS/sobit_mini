#!/usr/bin/env python
# coding:utf-8
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from subprocess import Popen
body_lift_joint = 0.05
body_roll_joint = 0
head_tilt_joint = 0
head_pan_joint = 0
right_shoulder_roll_joint = 0
right_shoulder_flex_joint = 0.40
right_wrist_flex_joint = 0
right_hand_motor_joint = 0
left_shoulder_roll_joint = 0
left_shoulder_flex_joint = -0.40
left_wrist_flex_joint = 0
left_hand_motor_joint = 0
class InitialJointPublisher:
	def __init__(self):
		self.pub_body_control = rospy.Publisher('/body_trajectory_controller/command', JointTrajectory, queue_size=10)
		self.pub_head_control = rospy.Publisher('/head_trajectory_controller/command', JointTrajectory, queue_size=10)
		self.pub_left_arm_control = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
		self.pub_right_arm_control = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=10)

	def check_publishers_connection(self, publisher):
			loop_rate_to_check_connection = rospy.Rate(1)
			while (publisher.get_num_connections() == 0 and not rospy.is_shutdown()):
				try:
					loop_rate_to_check_connection.sleep()
				except rospy.ROSInterruptException:
					pass
    
	def move_body_joint(self, joint_name, rad, time_from_start):
		point = JointTrajectoryPoint()
		point.positions.append(rad)
		point.time_from_start = rospy.Duration(time_from_start)
		traj = JointTrajectory()
		traj.joint_names.append(joint_name)
		traj.points.append(point)
		self.check_publishers_connection(self.pub_body_control)
		self.pub_body_control.publish(traj)
	
	def move_head_joint(self, joint_name, rad, time_from_start):
		point = JointTrajectoryPoint()
		point.positions.append(rad)
		point.time_from_start = rospy.Duration(time_from_start)
		traj = JointTrajectory()
		traj.joint_names.append(joint_name)
		traj.points.append(point)
		self.check_publishers_connection(self.pub_head_control)
		self.pub_head_control.publish(traj)

	def move_left_arm_joint(self, joint_name, rad, time_from_start):
		point = JointTrajectoryPoint()
		point.positions.append(rad)
		point.time_from_start = rospy.Duration(time_from_start)
		traj = JointTrajectory()
		traj.joint_names.append(joint_name)
		traj.points.append(point)
		self.check_publishers_connection(self.pub_left_arm_control)
		self.pub_left_arm_control.publish(traj)

	def move_right_arm_joint(self, joint_name, rad, time_from_start):
		point = JointTrajectoryPoint()
		point.positions.append(rad)
		point.time_from_start = rospy.Duration(time_from_start)
		traj = JointTrajectory()
		traj.joint_names.append(joint_name)
		traj.points.append(point)
		self.check_publishers_connection(self.pub_right_arm_control)
		self.pub_right_arm_control.publish(traj)

if __name__ == '__main__':
	rospy.init_node('intial_joint_publisher_node')
	initial = InitialJointPublisher()
	rospy.sleep(2)
	time_from_start = 1.0
	initial.move_body_joint('body_lift_joint', body_lift_joint, time_from_start)
	initial.move_body_joint('body_roll_joint', body_roll_joint, time_from_start)
	initial.move_head_joint('head_tilt_joint', head_tilt_joint, time_from_start)
	initial.move_head_joint('head_pan_joint', head_pan_joint, time_from_start)
	initial.move_right_arm_joint('right_shoulder_roll_joint', right_shoulder_roll_joint, time_from_start)
	initial.move_right_arm_joint('right_shoulder_flex_joint', right_shoulder_flex_joint, time_from_start)
	initial.move_right_arm_joint('right_wrist_flex_joint', right_wrist_flex_joint, time_from_start)
	initial.move_right_arm_joint('right_hand_motor_joint', right_hand_motor_joint, time_from_start)
	initial.move_left_arm_joint('left_shoulder_roll_joint', left_shoulder_roll_joint, time_from_start)
	initial.move_left_arm_joint('left_shoulder_flex_joint', left_shoulder_flex_joint, time_from_start)
	initial.move_left_arm_joint('left_wrist_flex_joint', left_wrist_flex_joint, time_from_start)
	initial.move_left_arm_joint('left_hand_motor_joint', left_hand_motor_joint, time_from_start)
	Popen(['rosnode','kill','/intial_joint_publisher_node'])