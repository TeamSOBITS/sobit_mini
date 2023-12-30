#!/usr/bin/env python3
# coding: utf-8
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg

class PS3_control:
    def __init__(self):
        # self.pub_joy = rospy.Publisher('/joy', sensor_msgs.msg.Joy, self.publisher_joy, queue_size=10)
        
        self.sub_joy = rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.subscribe_joy, queue_size=10)
        self.sub_joint_state = rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.subscribe_joint_state, queue_size=10)
        self.pub_body_control = rospy.Publisher('/body_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_head_control = rospy.Publisher('/head_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_left_arm_control = rospy.Publisher('/l_arm_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_right_arm_control = rospy.Publisher('/r_arm_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_wheel_control = rospy.Publisher('/mobile_base/commands/velocity',geometry_msgs.msg.Twist,queue_size=10)
        #rate
        self.rate = rospy.Rate(10)
        #subscriberのメッセージを受け取る変数
        self.joint_state_msg = sensor_msgs.msg.JointState()
        self.joy_button = [0] * 17
        self.left_joystick_lr = 0
        self.left_joystick_ud = 0
        self.right_joystick_lr = 0
        self.right_joystick_ud = 0
        self.magnifications = 0.3
        self.joint_pub_time = 0.001

    def subscribe_joint_state(self, msg):
        if "wheel" in msg.name[0]:
            pass
        else:
            self.joint_state_msg = msg

    def subscribe_joy(self, msg):
        self.joy_button = msg.buttons
        #print self.joy_button
        self.left_joystick_lr = msg.axes[0] * self.magnifications
        self.left_joystick_ud = msg.axes[1] * self.magnifications
        self.right_joystick_lr = msg.axes[3] * self.magnifications
        self.right_joystick_ud = msg.axes[4] * self.magnifications

    def check_publishers_connection(self, publisher):
        loop_rate_to_check_connection = rospy.Rate(1)
        while (publisher.get_num_connections() == 0 and not rospy.is_shutdown()):
            try:
                loop_rate_to_check_connection.sleep()
            except rospy.ROSInterruptException:
                pass

    def move_body_joint(self, joint_name, rad, time_from_start):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(rad)
        point.time_from_start = rospy.Duration(time_from_start)
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names.append(joint_name)
        traj.points.append(point)
        self.check_publishers_connection(self.pub_body_control)
        self.pub_body_control.publish(traj)
    
    def move_head_joint(self, joint_name, rad, time_from_start):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(rad)
        point.time_from_start = rospy.Duration(time_from_start)
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names.append(joint_name)
        traj.points.append(point)
        self.check_publishers_connection(self.pub_head_control)
        self.pub_head_control.publish(traj)

    def move_left_arm_joint(self, joint_name, rad, time_from_start):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(rad)
        point.time_from_start = rospy.Duration(time_from_start)
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names.append(joint_name)
        traj.points.append(point)
        self.check_publishers_connection(self.pub_left_arm_control)
        self.pub_left_arm_control.publish(traj)

    def move_right_arm_joint(self, joint_name, rad, time_from_start):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(rad)
        point.time_from_start = rospy.Duration(time_from_start)
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names.append(joint_name)
        traj.points.append(point)
        self.check_publishers_connection(self.pub_right_arm_control)
        self.pub_right_arm_control.publish(traj)
    
    def move_wheel(self, liner, angular):
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = self.left_joystick_ud * liner
        twist.angular.z = self.left_joystick_lr * angular
        self.check_publishers_connection(self.pub_wheel_control)
        self.pub_wheel_control.publish(twist)

    def reset_joint(self):
        self.move_body_joint("body_lift_joint", 0.05, self.joint_pub_time)
        self.move_body_joint("body_roll_joint", 0.0, self.joint_pub_time)
        self.move_head_joint("head_tilt_joint", 0.0, self.joint_pub_time)
        self.move_head_joint("head_pan_joint", 0.0, self.joint_pub_time)
        self.move_right_arm_joint("right_shoulder_roll_joint", 0.0, self.joint_pub_time)
        self.move_right_arm_joint("right_shoulder_flex_joint", 0.40, self.joint_pub_time)
        self.move_right_arm_joint("right_wrist_flex_joint", 0.0, self.joint_pub_time)
        self.move_right_arm_joint("right_hand_motor_joint", 0.0, self.joint_pub_time)
        self.move_left_arm_joint("left_shoulder_roll_joint", 0.0, self.joint_pub_time)
        self.move_left_arm_joint("left_shoulder_flex_joint", -0.4, self.joint_pub_time)
        self.move_left_arm_joint("left_wrist_flex_joint", 0.0, self.joint_pub_time)
        self.move_left_arm_joint("left_hand_motor_joint", 0.0, self.joint_pub_time)
    def pub_joy(self):
        while not rospy.is_shutdown():
            if self.joy_button[8] == True:#L2ボタンが押される
                rospy.loginfo("首を動かすモード")
                self.move_head_joint("head_tilt_joint", self.left_joystick_ud + self.joint_state_msg.position[3], self.joint_pub_time)
                self.move_head_joint("head_pan_joint", self.left_joystick_lr + self.joint_state_msg.position[2], self.joint_pub_time)
            #腰を動かす
            elif self.joy_button[9] == True:#R2ボタンが押される
                rospy.loginfo("腰を動かすモード")
                self.move_body_joint("body_roll_joint", self.left_joystick_lr + self.joint_state_msg.position[1], self.joint_pub_time)
                #self.move_body_joint("body_lift_joint", self.left_joystick_ud * 0.5 + self.joint_state_msg.position[0], self.joint_pub_time)
            #左手を動かす
            elif self.joy_button[10] == True:#L1ボタンが押される
                rospy.loginfo("左手を動かすモード")
                if self.joy_button[14] == True:#×ボタンが押される
                    self.move_left_arm_joint("left_hand_motor_joint", self.left_joystick_ud + self.joint_state_msg.position[4], self.joint_pub_time)
                elif self.joy_button[13] == True:#○ボタンが押される
                    self.move_left_arm_joint("left_wrist_flex_joint", self.left_joystick_ud + self.joint_state_msg.position[7], self.joint_pub_time)
                elif self.joy_button[12] == True:#△ボタンが押される
                    self.move_left_arm_joint("left_shoulder_flex_joint", self.left_joystick_ud + self.joint_state_msg.position[5], self.joint_pub_time)
                elif self.joy_button[15] == True:#□ボタンが押される
                    self.move_left_arm_joint("left_shoulder_roll_joint", self.left_joystick_ud + self.joint_state_msg.position[6], self.joint_pub_time)
            #右手を動かす
            elif self.joy_button[11] == True:#R1ボタンが押される
                if self.joy_button[14] == True:#×ボタンが押される
                    rospy.loginfo("右指を動かすモード")
                    #print("right_hand_motor_joint :  ", self.left_joystick_ud," + " ,self.joint_state_msg.position[8], " = ", self.left_joystick_ud + self.joint_state_msg.position[8])
                    self.move_right_arm_joint("right_hand_motor_joint", self.left_joystick_ud + self.joint_state_msg.position[8], self.joint_pub_time)
                elif self.joy_button[13] == True:#○ボタンが押される
                    #print("joint9 :  ", self.left_joystick_ud," + ", self.joint_state_msg.position[6], " = ", self.left_joystick_ud + self.joint_state_msg.position[6])
                    self.move_right_arm_joint("right_wrist_flex_joint", self.left_joystick_ud + self.joint_state_msg.position[11], self.joint_pub_time)
                elif self.joy_button[12] == True:#△ボタンが押される
                    #print("joint8 :  ", self.left_joystick_ud," + ", self.joint_state_msg.position[5], " = ", self.left_joystick_ud + self.joint_state_msg.position[5])
                    self.move_right_arm_joint("right_shoulder_flex_joint", self.left_joystick_ud + self.joint_state_msg.position[9], self.joint_pub_time)
                elif self.joy_button[15] == True:#□ボタンが押される
                    #print("joint7 :  ",  self.left_joystick_ud," + ", self.joint_state_msg.position[4], " = ", self.left_joystick_ud + self.joint_state_msg.position[4])
                    self.move_right_arm_joint("right_shoulder_roll_joint", self.left_joystick_ud + self.joint_state_msg.position[10], self.joint_pub_time)
            elif self.joy_button[16]:
                self.reset_joint()
            elif self.left_joystick_lr == 0 and self.left_joystick_ud == 0 and self.right_joystick_lr == 0 and self.right_joystick_ud == 0:
                #print("not publish")
                pass
            #足を動かすモード
            else:
                rospy.loginfo("足を動かすモード")
                self.move_wheel(0.2, 0.8)
            self.rate.sleep()




if __name__ == '__main__':
    rospy.init_node('sobit_mini_ps3_control_node')
    jc = PS3_control()
    jc.pub_joy()
    rospy.spin()
