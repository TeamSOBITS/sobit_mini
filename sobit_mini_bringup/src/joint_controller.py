#!/usr/bin/env python
# coding: utf-8
import rospy
import tf
import math
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sobit_common_msg.srv import gripper_ctrl
from sobit_common_msg.srv import gripper_ctrlResponse
from sobit_common_msg.srv import gripper_move
from sobit_common_msg.srv import gripper_moveResponse
from sobit_common_msg.srv import robot_motion
from sobit_common_msg.srv import robot_motionResponse
from sobit_common_msg.srv import odom_base
#import trajectory_msgs.msg


class JointController:
	def __init__(self):
		self.pub_body_control = rospy.Publisher('/body_trajectory_controller/command', JointTrajectory, queue_size=10)
		self.pub_head_control = rospy.Publisher('/head_trajectory_controller/command', JointTrajectory, queue_size=10)
		self.pub_left_arm_control = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
		self.pub_right_arm_control = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
		self.servise = rospy.Service("gripper_open_and_close", gripper_ctrl, self.open_and_close_gripper_server)
		self.servise = rospy.Service("gripper_move_to_target", gripper_move, self.move_gripper_to_target_server)
		self.servise = rospy.Service("motion_ctrl", robot_motion, self.move_to_registered_motion_server)
		self.listener = tf.TransformListener()
		self.from_base_to_arm_flex_link_x_cm = 13.5
		self.from_base_to_arm_flex_link_z_cm = 7.6
		self.arm_flex_link_x_cm = 2.4
		self.arm_flex_link_z_cm = 14.8
		self.wrist_flex_link_z_cm = 15.0
		self.can_grasp_min_x_cm = 20.0
		self.can_grasp_max_x_cm = 42.0
		self.can_grasp_min_z_cm = -8.0
		self.can_grasp_max_z_cm = 37.0

	def move_gripper_to_target_server(self, req_msg):
		target_object = req_msg.target_name
		key = self.listener.canTransform('/base_arm_link', target_object, rospy.Time(0))  # 座標変換の可否判定
		if not key:
			rospy.logerr("gripper_move_to_target Can't Transform [%s]", target_object)
			return gripper_moveResponse(False)

		rospy.loginfo("Gripper_move_to_target Target_object [%s]", target_object)
		(trans, rot) = self.listener.lookupTransform('/base_arm_link', target_object, rospy.Time(0))

		tan_rad = math.atan((trans[1] + req_msg.shift.y) / trans[0])
		sx = math.cos(tan_rad) * req_msg.shift.x
		sy = math.sin(tan_rad) * req_msg.shift.x
		object_x_cm = (trans[0] + sx) * 100
		object_y_cm = (trans[1] + sy) * 100
		object_z_cm = (trans[2] + req_msg.shift.z) * 100
		if object_z_cm < self.can_grasp_min_z_cm or object_z_cm > self.can_grasp_max_z_cm:
			return gripper_moveResponse(False)

		if self.from_base_to_arm_flex_link_z_cm < object_z_cm < self.from_base_to_arm_flex_link_z_cm + self.arm_flex_link_z_cm:
			print "objectがarm_flex_linkより低い場合"
			elbow_flex_joint_sin = (self.from_base_to_arm_flex_link_z_cm + self.arm_flex_link_z_cm - object_z_cm) / self.wrist_flex_link_z_cm
			elbow_flex_joint_rad = math.asin(elbow_flex_joint_sin)
			wrist_flex_joint_rad = - math.asin(elbow_flex_joint_sin)
			arm_flex_joint_rad = 0.0

		elif self.from_base_to_arm_flex_link_z_cm < self.from_base_to_arm_flex_link_z_cm + self.arm_flex_link_z_cm < object_z_cm:
			print "objectがarm_flex_linkより高い場合"
			elbow_flex_joint_sin = (object_z_cm - (self.from_base_to_arm_flex_link_z_cm + self.arm_flex_link_z_cm)) / self.wrist_flex_link_z_cm
			elbow_flex_joint_rad = - math.asin(elbow_flex_joint_sin)
			wrist_flex_joint_rad = math.asin(elbow_flex_joint_sin)
			arm_flex_joint_rad = 0.0

		elif object_z_cm < self.from_base_to_arm_flex_link_z_cm:
			print "objectがfrom_base_to_arm_flex_linkより低い場合"
			elbow_flex_joint_rad = math.atan((- object_z_cm + self.from_base_to_arm_flex_link_x_cm) / self.wrist_flex_link_z_cm) - math.radians(90.0)
			wrist_flex_joint_rad = - math.atan((- object_z_cm + self.from_base_to_arm_flex_link_x_cm) / self.wrist_flex_link_z_cm)
			arm_flex_joint_rad = 1.57

		time_from_start = 0.1
		#self.move_arm_joint("arm_roll_joint", 0.0, time_from_start)
		rospy.sleep(time_from_start)
		#self.move_arm_joint("arm_flex_joint", arm_flex_joint_rad, time_from_start)
		rospy.sleep(time_from_start)
		#self.move_arm_joint("elbow_flex_joint", elbow_flex_joint_rad, time_from_start)
		rospy.sleep(time_from_start)
		#self.move_arm_joint("wrist_flex_joint", wrist_flex_joint_rad, time_from_start)
		rospy.sleep(time_from_start)

		turning_deg = math.atan(object_y_cm / object_x_cm) * 57.2958
		str_turning_deg = "T:" + str(turning_deg)
		self.move_wheel(str_turning_deg)
		rospy.sleep(2)

		from_base_to_hand_motor_link = self.from_base_to_arm_flex_link_x_cm + self.arm_flex_link_x_cm + self.wrist_flex_link_z_cm * math.cos(elbow_flex_joint_rad)
		moving_cm = math.sqrt(object_x_cm ** 2 + object_y_cm ** 2) - from_base_to_hand_motor_link
		str_moving_cm = "S:" + str(moving_cm)
		self.move_wheel(str_moving_cm)

		return gripper_moveResponse(True)

	def open_and_close_gripper_server(self, req_msg):
		hand_motor_joint_rad = req_msg.rad
		time_from_start = 0.1
		#self.move_arm_joint("hand_motor_joint", hand_motor_joint_rad, time_from_start)
		rospy.sleep(time_from_start)
		return gripper_ctrlResponse(True)

	def move_to_registered_motion_server(self, req_msg):
		motion_type = req_msg.motion_type
		if motion_type == "INITIAL_POSE":
			self.move_to_initial_pose()
		elif motion_type == "HOLDING_POSE":
			self.move_to_holding_pose()
		elif motion_type == "DETECTING_POSE":
			self.move_to_detecting_pose()
		return robot_motionResponse(True)

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

	def move_wheel(self, str_distance):
		rospy.wait_for_service('/robot_ctrl/odom_base_ctrl')
		try:
			wheel_ctrl_service = rospy.ServiceProxy('/robot_ctrl/odom_base_ctrl', odom_base)
			res = wheel_ctrl_service(str_distance)
			return res.res_str
		except rospy.ServiceException as e:
			print "Service call failed: %s" % e

	def move_to_initial_pose(self):
		time_from_start = 0.1
		"""  
		self.move_arm_joint("arm_roll_joint", 0.00, time_from_start)
		rospy.sleep(time_from_start)
		self.move_arm_joint("arm_flex_joint", 0.00, time_from_start)
		rospy.sleep(time_from_start)
		self.move_arm_joint("elbow_flex_joint", 1.31, time_from_start)
		rospy.sleep(time_from_start)
		self.move_arm_joint("wrist_flex_joint", 0.00, time_from_start)
		rospy.sleep(time_from_start)
		self.move_arm_joint("hand_motor_joint", 0.00, time_from_start)
		rospy.sleep(time_from_start)
		self.move_xtion_joint("xtion_tilt_joint", 0.00, time_from_start)
		"""
		rospy.sleep(time_from_start)

	def move_to_holding_pose(self):
		time_from_start = 0.1
		"""
		self.move_arm_joint("arm_roll_joint", 0.00, time_from_start)
		rospy.sleep(time_from_start)
		self.move_arm_joint("arm_flex_joint", 0.00, time_from_start)
		rospy.sleep(time_from_start)
		self.move_arm_joint("elbow_flex_joint", 1.31, time_from_start)
		rospy.sleep(time_from_start)
		self.move_arm_joint("wrist_flex_joint", -1.31, time_from_start)
		rospy.sleep(time_from_start)
		self.move_arm_joint("hand_motor_joint", 0.00, time_from_start)
		rospy.sleep(time_from_start)
		self.move_xtion_joint("xtion_tilt_joint", 0.00, time_from_start)
		"""
		rospy.sleep(time_from_start)
	
	def move_to_detecting_pose(self):
		time_from_start = 0.1
		"""
		self.move_arm_joint("arm_roll_joint", 0.00, time_from_start)
		#rospy.sleep(time_from_start)
		self.move_arm_joint("arm_flex_joint", 0.00, time_from_start)
		#rospy.sleep(time_from_start)
		self.move_arm_joint("elbow_flex_joint", 1.31, time_from_start)
		#rospy.sleep(time_from_start)
		self.move_arm_joint("wrist_flex_joint", 0.00, time_from_start)
		#rospy.sleep(time_from_start)
		self.move_arm_joint("hand_motor_joint", 0.00, time_from_start)
		#rospy.sleep(time_from_start)
		self.move_xtion_joint("xtion_tilt_joint", 0.53, time_from_start)
		"""
		rospy.sleep(time_from_start)

if __name__ == "__main__":
	rospy.init_node("joint_controller")
	JointController()
	rospy.spin()
