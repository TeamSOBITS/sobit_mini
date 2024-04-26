#!/usr/bin/env python3
# coding: utf-8

import rospy

from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import geometry_msgs.msg

class JoyControl:
    def __init__(self):
        # Subscriber
        self.sub_joy         = rospy.Subscriber('/joy', Joy, self.subscribe_joy, queue_size=10)
        self.sub_joint_state = rospy.Subscriber('/joint_states', JointState, self.subscribe_joint_state, queue_size=10)
        
        # Publisher
        self.pub_body_control      = rospy.Publisher('/body_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.pub_head_control      = rospy.Publisher('/head_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.pub_left_arm_control  = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.pub_right_arm_control = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.pub_wheel_control     = rospy.Publisher('/cmd_vel_mux/input/teleop',geometry_msgs.msg.Twist,queue_size=10)

        # Rate
        self.rate = rospy.Rate(10)

        # Parameters
        self.joint_state_msg = JointState()
        self.joy_button = [0] * 17
        self.left_joystick_lr = 0
        self.left_joystick_ud = 0
        self.right_joystick_lr = 0
        self.right_joystick_ud = 0
        self.magnifications = 0.2
        self.joint_pub_time = 0.001

    def subscribe_joint_state(self, msg):
        if "wheel" in msg.name[0]:
            pass
        else:
            self.joint_state_msg = msg

    def subscribe_joy(self, msg):
        self.joy_button = msg.buttons

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
            # L2 button pressed: Move the head
            if self.joy_button[6] == True:
                rospy.loginfo("Mode: head control")
                self.move_head_joint("head_tilt_joint", self.left_joystick_ud + self.joint_state_msg.position[3], self.joint_pub_time)
                self.move_head_joint("head_pan_joint", self.left_joystick_lr + self.joint_state_msg.position[2], self.joint_pub_time)
  
            # R2 button pressed: Move the body
            elif self.joy_button[7] == True:
                rospy.loginfo("Mode: body control")
                self.move_body_joint("body_roll_joint", self.left_joystick_lr + self.joint_state_msg.position[1], self.joint_pub_time)
            
            # L1 button pressed: Move the left arm
            elif self.joy_button[4] == True:
                rospy.loginfo("Mode: left arm control")

                # x button pressed
                if self.joy_button[0] == True:
                    self.move_left_arm_joint("left_hand_motor_joint", self.left_joystick_ud + self.joint_state_msg.position[4], self.joint_pub_time)
                
                # circle button pressed
                elif self.joy_button[1] == True:
                    self.move_left_arm_joint("left_wrist_flex_joint", self.left_joystick_ud + self.joint_state_msg.position[7], self.joint_pub_time)
                
                # triangle button pressed
                elif self.joy_button[2] == True:
                    self.move_left_arm_joint("left_shoulder_flex_joint", self.left_joystick_ud + self.joint_state_msg.position[5], self.joint_pub_time)
                
                # square button pressed
                elif self.joy_button[3] == True:
                    self.move_left_arm_joint("left_shoulder_roll_joint", self.left_joystick_ud + self.joint_state_msg.position[6], self.joint_pub_time)

                # R1 button pressed: Move the right arm
                elif self.joy_button[5] == True:
                    rospy.loginfo("Mode: right arm control")
                    # x button pressed
                    if self.joy_button[0] == True:
                        self.move_right_arm_joint("right_hand_motor_joint", self.left_joystick_ud + self.joint_state_msg.position[8], self.joint_pub_time)
                    
                    # circle button pressed
                    elif self.joy_button[1] == True:
                        self.move_right_arm_joint("right_wrist_flex_joint", self.left_joystick_ud + self.joint_state_msg.position[11], self.joint_pub_time)
                    
                    # triangle button pressed
                    elif self.joy_button[2] == True:#△ボタンが押される
                        self.move_right_arm_joint("right_shoulder_flex_joint", self.left_joystick_ud + self.joint_state_msg.position[9], self.joint_pub_time)
                    
                    # square button pressed
                    elif self.joy_button[3] == True:#□ボタンが押される
                        self.move_right_arm_joint("right_shoulder_roll_joint", self.left_joystick_ud + self.joint_state_msg.position[10], self.joint_pub_time)

            # Start button pressed: Initialize the robot
            elif self.joy_button[8] == True:
                self.reset_joint()

            elif self.left_joystick_lr == 0 and self.left_joystick_ud == 0 and self.right_joystick_lr == 0 and self.right_joystick_ud == 0:
                pass

            # Move the legs
            else:
                rospy.loginfo("Mode: legs control")
                self.move_wheel(0.2, 1.0)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('sobit_mini_ps3_control_node')
    jc = JoyControl()
    jc.pub_joy()
    rospy.spin()
