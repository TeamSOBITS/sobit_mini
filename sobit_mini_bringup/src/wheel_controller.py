#!/usr/bin/env python
# coding: utf-8
import rospy
import tf
import math
import time
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from turtlebot_edu.srv import *

#indent = 1 tabs
# S:1000	(1000cm直進)
# T:90		(90度回転、反時計回り正回転)

###############################################################################

class OdomBaseController:
	def __init__(self):

		self.current_pose = Point()

		#回転制御パラメータ
		self.turn_speed = 0 		#初速度(初期：0)
		self.turn_acs = 1	#加速度(初期：0.5)
		self.turn_speed_max = 150	#最高速度(初期：20)
		self.turn_speed_min = 0
		self.turn_Ki = 0.15

		#直進制御パラメータ
		self.stlight_speed = 0 			#初速度(初期：0m/s)
		self.stlight_acs = 0.01 		#加速度(デフォルト：0.01)
		self.stlight_speed_max = 0.4 	#最高速度(デフォルト：0.2m/s)
		self.stlight_speed_min = 0		#最低速度(デフォルト:0m/s)
		self.stlight_Ki = 0.1			#積分係数


		#初期化
		self.stlight_speed_before = 0
		self.current_stlight_speed = 0
		self.current_turn_speed = 0
		self.move_order_T = False
		self.move_order_S = False
		self.start_pose = Point()
		self.order_deg = 0
		self.moved_deg = 0
		self.order_cm = 0
		self.current_distance = 0
		self.error_S_P = 0
		self.error_S_I = 0
		self.error_T_P = 0
		self.error_T_I = 0

		self.pub_twist = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
		self.sub_odom = rospy.Subscriber('/odom',Odometry, self.odom_callback)
		self.sub_string = rospy.Service('odom_base_ctrl', odom_base, self.string_callback)
		self.pub_output_log = rospy.Publisher('/odom_base/output_log', String, queue_size=10)

		rospy.loginfo("odom_base_controller is OK.")

	def string_callback(self,req):
		if self.move_order_T == True or self.move_order_S == True:
			rospy.loginfo("Sorry,I can't do it.")
			return False
		check = self.Check_Command(req)
		if check == False:
			rospy.loginfo("The command is fail.")
			return False
		else:
			self.start_pose = Point(self.current_pose.x,self.current_pose.y,self.current_pose.z)

			if "T" in req.req_str:
				self.order_deg = self.Read_Value(req)
				self.move_order_T = True

				rospy.loginfo("order: Trun: %f(deg)" % (self.order_deg))


			elif "S" in req.req_str:
				self.order_cm = self.Read_Value(req)#cm
				self.move_order_S = True
				rospy.loginfo("order: Straight: %f(cm)" % (self.order_cm))

		while True:
			time.sleep(0.1)
			if self.move_order_T == False and self.move_order_S == False:
				break

		rospy.loginfo("Moving Finished")
		return str(req.req_str) + "_finished"


	def odom_callback(self, odom):
		euler = tf.transformations.euler_from_quaternion((odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w))

		yaw = euler[2]
		yaw = math.degrees(yaw)

		self.current_pose.x = odom.pose.pose.position.x * 100#[cm]
		self.current_pose.y = odom.pose.pose.position.y * 100#[cm]
		self.current_pose.z = yaw#[deg]

#		rospy.loginfo("x: %f[cm] y=%f[cm] yaw=%f[deg]",self.current_pose.x, self.current_pose.y, yaw )
		send_cmd = Twist()#メッセージ変数の宣言
		if self.move_order_T == True and self.move_order_S == False:# T　回転実行

			#rospy.loginfo("Trun: %f(deg) self.moved_deg=%f[deg]",self.order_deg , self.moved_deg)

			#加速区間
			if self.moved_deg < abs(self.order_deg) / 5:
				#self.current_stlight_acs += self.stlight_acs
				self.turn_speed = self.turn_speed + self.turn_acs
				self.current_turn_speed = self.turn_speed
			#減速区間
			elif self.moved_deg > abs(self.order_deg) * 4 / 5:
				#PI制御
				self.turn_speed_before = self.turn_speed	#前回パブリッシュした速度を保存
				if self.turn_speed > self.error_T_I:    #現在のスピードが積分係数より小さい時
					self.error_T_P = (abs(self.order_deg) - self.moved_deg) / (abs(self.order_deg) / 5)  #指定した距離に対する現在走行距離の割合
					self.turn_speed = self.current_turn_speed * self.error_T_P + self.error_T_I
					self.error_T_I += (self.turn_speed_before - self.turn_speed)*self.turn_Ki
					#print "error_T_I=%f" % self.error_T_I
				elif self.turn_speed <= self.error_T_I:
					self.turn_speed = self.error_T_I
				else:
					pass
			#等速区間
			else:
				pass

			#最高速度補正
			if self.turn_speed >= self.turn_speed_max:
				self.turn_speed = self.turn_speed_max
			#逆走防止
			if self.turn_speed < self.turn_speed_min:
				self.turn_speed = self.turn_speed_min


			#print('speed:' + str(self.turn_speed))

			if self.order_deg > 0:
					send_cmd.angular.z = math.radians(self.turn_speed)
			else:#時計回り
					send_cmd.angular.z = math.radians(-self.turn_speed)

			if self.moved_deg < abs(self.order_deg):
				self.pub_twist.publish(send_cmd)
				sub_point = abs(self.current_pose.z - self.start_pose.z)
				if sub_point > 180:
					sub_point = abs(sub_point - 360)
				self.moved_deg += sub_point
				self.start_pose = Point(self.current_pose.x, self.current_pose.y, self.current_pose.z)
				#time.sleep(0.03)
			else:
				self.pub_twist.publish(Twist())#停止
				self.is_move = False
				rospy.loginfo("Trun finished")
				self.turn_speed = 0
				self.current_turn_speed = 0
				self.order_deg = 0
				self.error_T_P = 0
				self.error_T_I = 0
				self.move_order_T = False
				self.moved_deg = 0
				output_log = "T:" + str(self.order_deg) + "_finished"
				self.pub_output_log.publish(output_log)



		elif self.move_order_T == False and self.move_order_S == True:# S　直進実行

			#rospy.loginfo("Straight: %f(cm) self.current_distance=%f[cm]" , self.order_cm , self.current_distance)

			#加速区間
			if self.current_distance < abs(self.order_cm) / 5:
				#self.current_stlight_acs += self.stlight_acs
				self.stlight_speed = self.stlight_speed + self.stlight_acs
				self.current_stlight_speed = self.stlight_speed
			#減速区間
			elif self.current_distance > abs(self.order_cm) * 4 / 5:
				#PI制御
				self.stlight_speed_before = self.stlight_speed	#前回パブリッシュした速度を保存
				if self.stlight_speed > self.error_S_I:    #現在のスピードが積分係数より小さい時
					self.error_S_P = (abs(self.order_cm) - self.current_distance) / (abs(self.order_cm) / 5)  #指定した距離に対する現在走行距離の割合
					self.stlight_speed = self.current_stlight_speed * self.error_S_P + self.error_S_I
					self.error_S_I += (self.stlight_speed_before - self.stlight_speed)*self.stlight_Ki
					#print "error_S_I=%f" % self.error_S_I
				elif self.stlight_speed <= self.error_S_I:
					self.stlight_speed = self.error_S_I
				else:
					pass
			#等速区間
			else:
				pass

			#最高速度補正
			if self.stlight_speed >= self.stlight_speed_max:
				self.stlight_speed = self.stlight_speed_max
			#逆走防止
			if self.stlight_speed < self.stlight_speed_min:
				self.stlight_speed = self.stlight_speed_min

			#print('speed:' + str(self.stlight_speed))

			if self.order_cm > 0:
				send_cmd.linear.x = self.stlight_speed #cm
			else:
				send_cmd.linear.x = -self.stlight_speed #cm

			if self.current_distance < abs(self.order_cm):
				self.pub_twist.publish(send_cmd)
				sub_x = self.start_pose.x - self.current_pose.x
				sub_y = self.start_pose.y - self.current_pose.y
				self.current_distance = math.hypot(sub_x,sub_y)
			else:
				self.pub_twist.publish(Twist())#停止
				rospy.loginfo("Straight finished")
				self.stlight_speed = 0
				self.current_stlight_speed = 0
				self.order_cm = 0
				self.error_S_P = 0
				self.error_S_I = 0
				self.move_order_S = False
				self.current_distance = 0
				output_log = "S:" + str(self.order_cm) + "_finished"
				self.pub_output_log.publish(output_log)

		elif self.move_order_T == True and self.move_order_S == True:# 例外なので停止
			rospy.loginfo("Sorry,I can't do it.")
			self.pub_twist.publish(Twist())#停止

	def Check_Command(self, line):
		#print("チェックコマンド!!!")
		cmd_line = line.req_str[0:2]
		value_line = line.req_str[2:len(line.req_str)]
		for i in range(len(cmd_line)):
			key = cmd_line[i]
			if key == 'T' or key == 'S' or key == ':':
				#print "Check ok"
				pass
			else:
				rospy.loginfo("check_command cmd error")
				rospy.loginfo(line.req_str)
				return False
		for i in range(len(value_line)):
			key = value_line[i]
			if key >= '0' and key <= '9' or key == '-' or key == '.':
				#print "Check ok"
				pass
			else:
				rospy.loginfo("check_command cmd error")
				rospy.loginfo(line.req_str)
				return False
		return True

	def Read_Value(self, line):
		#print("値の読み込み！！！")
		value_str = line.req_str[2:len(line.req_str)]
		value = float(value_str)
		return value

if __name__ == '__main__':

	rospy.init_node('odom_base_controller')

	obc = OdomBaseController()
	rospy.spin()
