#!/usr/bin/env python3
# coding: utf-8
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
from sobit_mini_module import SobitMiniJointController
import sys
import math

class PS3_control:
    def __init__(self):
        self.sub_joy = rospy.Subscriber('/joy', sensor_msgs.msg.Joy, self.subscribe_joy, queue_size=10)
        self.sub_joint_state = rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.subscribe_joint_state, queue_size=10)
        self.pub_body_control = rospy.Publisher('/body_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_head_control = rospy.Publisher('/head_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_left_arm_control = rospy.Publisher('/l_arm_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_right_arm_control = rospy.Publisher('/r_arm_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory, queue_size=10)
        self.pub_wheel_control = rospy.Publisher('/mobile_base/commands/velocity',geometry_msgs.msg.Twist,queue_size=10)
        #rate
        self.rate = rospy.Rate(10)
        #Variable to receive subscriber's message
        self.joint_state_msg = sensor_msgs.msg.JointState()
        self.joy_button = [0] * 17
        self.left_joystick_lr = 0
        self.left_joystick_ud = 0
        self.right_joystick_lr = 0
        self.right_joystick_ud = 0
        self.joint_pub_time = 0.001

    def subscribe_joint_state(self, msg):
        if "wheel" in msg.name[0]:
            pass
        else:
            self.joint_state_msg = msg

    def subscribe_joy(self, msg):
        self.joy_button = msg.buttons
        self.left_joystick_lr = msg.axes[0]
        self.left_joystick_ud = msg.axes[1]
        self.right_joystick_lr = msg.axes[3]
        self.right_joystick_ud = msg.axes[4]

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

    def move_l_arm_joint(self, joint_name, rad, time_from_start):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(rad)
        point.time_from_start = rospy.Duration(time_from_start)
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names.append(joint_name)
        traj.points.append(point)
        self.check_publishers_connection(self.pub_left_arm_control)
        self.pub_left_arm_control.publish(traj)

    def move_r_arm_joint(self, joint_name, rad, time_from_start):
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
        args = sys.argv
        mini_ctr = SobitMiniJointController(args[0]) # args[0] : Arguments for ros::init() on C++
        mini_ctr.moveToPose( "initial_pose" )
    def pub_joy(self):
        while not rospy.is_shutdown():
            if self.joy_button[6] == True:#L2 button is pressed
                rospy.loginfo("首を動かすモード")
                self.move_head_joint("head_tilt_joint", self.left_joystick_ud * 0.45, self.joint_pub_time)
                self.move_head_joint("head_pan_joint", self.left_joystick_lr * 1.57, self.joint_pub_time)
                print("head_tilt_joint_rad:",self.left_joystick_ud * 0.45)
                print("head_pan_joint_rad:",self.left_joystick_lr * 1.57)
                print("head_tilt_joint_deg:", math.degrees(self.left_joystick_ud * 0.45))
                print("head_pan_joint_deg:", math.degrees(self.left_joystick_lr * 1.57))
            #move one's hips
            elif self.joy_button[7] == True:#R2 button is pressed
                rospy.loginfo("腰を動かすモード")
                self.move_body_joint("body_roll_joint", -self.left_joystick_lr * math.radians(237), self.joint_pub_time)
                print("body_roll_joint_rad:", (self.left_joystick_lr * math.radians(237)) * (22/116))
                print("body_roll_joint_deg:", math.degrees((self.left_joystick_lr * math.radians(237)) * (22/116)))
            #move your left hand
            elif self.joy_button[4] == True:#L1 button is pressed
                rospy.loginfo("左手を動かすモード")
                if self.joy_button[0] == True and self.joy_button[3] == True:#×ボタンと□ボタンが押される
                    self.move_l_arm_joint("l_hand_joint", -self.left_joystick_lr, self.joint_pub_time)
                    print("l_hand_joint_rad:",self.left_joystick_lr)
                    print("l_hand_joint_deg:",math.degrees(self.left_joystick_lr))
                elif self.joy_button[0] == True:#The x button is pressed
                    self.move_l_arm_joint("l_arm_wrist_tilt_joint", -self.left_joystick_ud * math.radians(90), self.joint_pub_time)
                    print("l_arm_wrist_tilt_joint_rad:",self.left_joystick_ud * math.radians(90))
                    print("l_arm_wrist_tilt_joint_deg:",math.degrees(self.left_joystick_ud * math.radians(90)))
                elif self.joy_button[1] == True:#The ○ button is pressed
                    self.move_l_arm_joint("l_arm_elbow_tilt_joint", -self.left_joystick_ud * math.radians(90), self.joint_pub_time)
                    print("l_arm_elbow_tilt_joint_rad:",self.left_joystick_ud * math.radians(90))
                    print("l_arm_elbow_tilt_joint_deg:",math.degrees(self.left_joystick_ud * math.radians(90)))
                elif self.joy_button[2] == True:#The △ button is pressed
                    self.move_l_arm_joint("l_arm_shoulder_pan_joint", self.left_joystick_ud * math.radians(90), self.joint_pub_time)
                    print("l_arm_shoulder_pan_joint_rad:",self.left_joystick_lr * math.radians(90))
                    print("l_arm_shoulder_pan_joint_deg:",math.degrees(self.left_joystick_ud * math.radians(90)))
                elif self.joy_button[3] == True:#The □ button is pressed
                    self.move_l_arm_joint("l_arm_shoulder_roll_joint", -self.left_joystick_ud * math.radians(90), self.joint_pub_time)
                    print("l_arm_shoulder_roll_joint_rad:",self.left_joystick_ud * math.radians(90))
                    print("l_arm_shoulder_roll_joint_deg:",math.degrees(self.left_joystick_ud * math.radians(90)))
                

            #Move your right hand
            elif self.joy_button[5] == True:#R1 button is pressed
                rospy.loginfo("右手を動かすモード")
                if self.joy_button[0] == True and self.joy_button[3] == True:#×ボタンと□ボタンが押される
                    self.move_r_arm_joint("r_hand_joint", self.left_joystick_lr, self.joint_pub_time)
                    print("r_hand_joint_rad:",self.left_joystick_lr)
                    print("r_hand_joint_deg:",math.degrees(self.left_joystick_lr))
                elif self.joy_button[0] == True:#The x button is pressed
                    self.move_r_arm_joint("r_arm_wrist_tilt_joint", self.left_joystick_ud * math.radians(90), self.joint_pub_time)
                    print("r_arm_wrist_tilt_joint_rad:",self.left_joystick_ud * math.radians(90))
                    print("r_arm_wrist_tilt_joint_deg:",math.degrees(self.left_joystick_ud * math.radians(90)))
                elif self.joy_button[1] == True:#The ○ button is pressed
                    self.move_r_arm_joint("r_arm_elbow_tilt_joint", self.left_joystick_ud * math.radians(90), self.joint_pub_time)
                    print("r_arm_elbow_tilt_joint_rad:",self.left_joystick_ud * math.radians(90))
                    print("r_arm_elbow_tilt_joint_deg:",math.degrees(self.left_joystick_ud * math.radians(90)))
                elif self.joy_button[2] == True:#The △ button is pressed
                    self.move_r_arm_joint("r_arm_shoulder_pan_joint", -self.left_joystick_ud * math.radians(90), self.joint_pub_time)
                    print("r_arm_shoulder_pan_joint_rad:",self.left_joystick_lr * math.radians(90))
                    print("r_arm_shoulder_pan_joint_deg:",math.degrees(self.left_joystick_ud * math.radians(90)))
                elif self.joy_button[3] == True:#The □ button is pressed
                    self.move_r_arm_joint("r_arm_shoulder_roll_joint", self.left_joystick_ud * math.radians(90), self.joint_pub_time)
                    print("r_arm_shoulder_roll_joint_rad:",self.left_joystick_ud * math.radians(90))
                    print("r_arm_shoulder_roll_joint_deg:",math.degrees(self.left_joystick_ud * math.radians(90)))
            elif self.joy_button[8] == True:
                self.reset_joint()
            elif self.left_joystick_lr == 0 and self.left_joystick_ud == 0 and self.right_joystick_lr == 0 and self.right_joystick_ud == 0:
                #print("not publish")
                pass
            #Leg Movement Mode
            else:
                rospy.loginfo("足を動かすモード")
                self.move_wheel(0.2, 1.0)
            self.rate.sleep()




if __name__ == '__main__':
    rospy.init_node('sobit_mini_ps3_control_node')
    jc = PS3_control()
    jc.pub_joy()
    rospy.spin()
