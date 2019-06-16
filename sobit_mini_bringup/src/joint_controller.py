#!/usr/bin/env python
# coding: utf-8
import rospy
import tf
import math
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from sobit_common_msg.srv import gripper_ctrl
from sobit_common_msg.srv import gripper_ctrlResponse
from sobit_common_msg.srv import gripper_move
from sobit_common_msg.srv import gripper_moveResponse
from sobit_common_msg.srv import robot_motion
from sobit_common_msg.srv import robot_motionResponse
from sobit_common_msg.srv import odom_base
from sobit_common_msg.srv import grasping_jedgment
from sobit_common_msg.srv import grasping_jedgmentResponse


class JointController:

    def __init__(self):
        self.pub_body_control = rospy.Publisher('/body_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.pub_head_control = rospy.Publisher('/head_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.pub_left_arm_control = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.pub_right_arm_control = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.servise = rospy.Service("right_gripper_move_to_target", gripper_move, self.move_right_gripper_to_target_server)
        self.servise = rospy.Service("left_gripper_move_to_target", gripper_move, self.move_left_gripper_to_target_server)
        self.servise = rospy.Service("right_gripper_open_and_close", gripper_ctrl, self.open_and_close_right_gripper_server)
        self.servise = rospy.Service("left_gripper_open_and_close", gripper_ctrl, self.open_and_close_left_gripper_server)
        self.servise = rospy.Service("motion_ctrl", robot_motion, self.move_to_registered_motion_server)
        self.servise = rospy.Service("right_hand_grasping_jedgment", grasping_jedgment, self.jedge_right_hand_grasping)
        self.servise = rospy.Service("left_hand_grasping_jedgment", grasping_jedgment, self.jedge_left_hand_grasping)
        self.listener = tf.TransformListener()
        self.body_control_data = JointTrajectory()
        self.head_control_data = JointTrajectory()
        self.left_arm_control_data = JointTrajectory()
        self.right_arm_control_data = JointTrajectory()
        self.body_control_data.points = [JointTrajectoryPoint()]
        self.head_control_data.points = [JointTrajectoryPoint()]
        self.left_arm_control_data.points = [JointTrajectoryPoint()]
        self.right_arm_control_data.points = [JointTrajectoryPoint()]
        self.joint_info = JointState()
        self.from_base_to_arm_flex_link_x_cm = 13.5
        self.from_base_to_arm_flex_link_z_cm = 7.6
        self.arm_flex_link_x_cm = 2.4
        self.arm_flex_link_z_cm = 14.8
        self.wrist_flex_link_z_cm = 15.0
        self.can_grasp_min_x_cm = 20.0
        self.can_grasp_max_x_cm = 42.0
        self.from_base_to_shoulder_z_cm = 72.0
        self.from_base_to_shoulder_x_cm = 15.4
        self.from_shoulder_to_wrist_x_cm = 19.0
        self.from_base_to_plate_middle_link_x_cm = 1.4
        self.can_grasp_min_z_cm = 48.0
        self.can_grasp_max_z_cm = 85.0

    def joint_states_callback(self, msg):
        if len(msg.name) == 12:
            self.joint_info = msg

    def move_right_gripper_to_target_server(self, req_msg):
        target_object = req_msg.target_name
        key = self.listener.canTransform('/base_footprint', target_object, rospy.Time(0))  # 座標変換の可否判定
        if not key:
            rospy.logerr("gripper_move_to_target Can't Transform [%s]", target_object)
            return gripper_moveResponse(False)

        rospy.loginfo("Gripper_move_to_target Target_object [%s]", target_object)
        (trans, rot) = self.listener.lookupTransform('/base_footprint', target_object, rospy.Time(0))

        tan_rad = math.atan((trans[1] + req_msg.shift.y) / trans[0])
        sx = math.cos(tan_rad) * req_msg.shift.x
        sy = math.sin(tan_rad) * req_msg.shift.x
        object_x_cm = (trans[0] + sx) * 100
        object_y_cm = (trans[1] + sy) * 100
        object_z_cm = (trans[2] + req_msg.shift.z) * 100
        if object_z_cm < self.can_grasp_min_z_cm or object_z_cm > self.can_grasp_max_z_cm:
            return gripper_moveResponse(False)
        time_from_start = 0.3
        self.move_body_joint('body_roll_joint', 1.57, time_from_start)
        self.move_head_joint('head_pan_joint', -1.57, time_from_start)
        rospy.sleep(10)
        # とりあえず右手で掴む処理を書く
        moving_cm = 0
        if self.from_base_to_shoulder_z_cm < object_z_cm:
            print "objectが肩の高さより高い場合"
            lift_cm = object_z_cm - self.from_base_to_shoulder_z_cm
            self.move_body_joint('body_lift_joint', lift_cm * 0.01, time_from_start)
            self.move_right_arm_joint('right_shoulder_flex_joint', 1.57, time_from_start)
            self.move_right_arm_joint('right_shoulder_roll_joint', 0.0, time_from_start)
            self.move_right_arm_joint('right_wrist_flex_joint', 0.0, time_from_start)
            moving_cm = math.sqrt(object_x_cm**2 + object_y_cm**2) - self.from_base_to_shoulder_x_cm - self.from_shoulder_to_wrist_x_cm
        else:
            print "objectが肩の高さより低い場合"
            from_shoulder_to_object_z = self.from_base_to_shoulder_z_cm - object_z_cm
            from_shoulder_to_object_x = math.sqrt(object_x_cm**2 + object_y_cm**2) - self.from_base_to_shoulder_x_cm
            right_shoulder_flex_joint_rad = 1.57 - math.atan2(from_shoulder_to_object_z, from_shoulder_to_object_x)
            right_wrist_flex_joint_rad = math.atan2(from_shoulder_to_object_z, from_shoulder_to_object_x)
            self.move_right_arm_joint('right_shoulder_flex_joint', right_shoulder_flex_joint_rad, time_from_start)
            self.move_right_arm_joint('right_wrist_flex_joint', right_wrist_flex_joint_rad, time_from_start)
            print 'right_shoulder_flex_joint : ', right_shoulder_flex_joint_rad
            print 'right_wrist_flex_joint : ', right_wrist_flex_joint_rad
            moving_cm = math.sqrt(object_x_cm**2 + object_y_cm**2) - self.from_base_to_shoulder_x_cm - self.from_shoulder_to_wrist_x_cm * math.cos(right_wrist_flex_joint_rad)

        turning_deg = math.atan(object_y_cm / object_x_cm) * 57.2958
        str_turning_deg = 'T:' + str(turning_deg)
        self.move_wheel(str_turning_deg)
        rospy.sleep(2)
        str_moving_cm = 'S:' + str(moving_cm)
        self.move_wheel(str_moving_cm)

        return gripper_moveResponse(True)

    def move_left_gripper_to_target_server(self, req_msg):
        target_object = req_msg.target_name
        key = self.listener.canTransform('/base_footprint', target_object, rospy.Time(0))  # 座標変換の可否判定
        if not key:
            rospy.logerr("gripper_move_to_target Can't Transform [%s]", target_object)
            return gripper_moveResponse(False)

        rospy.loginfo("Gripper_move_to_target Target_object [%s]", target_object)
        (trans, rot) = self.listener.lookupTransform('/base_footprint', target_object, rospy.Time(0))

        tan_rad = math.atan((trans[1] + req_msg.shift.y) / trans[0])
        sx = math.cos(tan_rad) * req_msg.shift.x
        sy = math.sin(tan_rad) * req_msg.shift.x
        object_x_cm = (trans[0] + sx) * 100 + self.from_base_to_plate_middle_link_x_cm
        object_y_cm = (trans[1] + sy) * 100
        object_z_cm = (trans[2] + req_msg.shift.z) * 100
        if object_z_cm < self.can_grasp_min_z_cm or object_z_cm > self.can_grasp_max_z_cm:
            return gripper_moveResponse(False)
        time_from_start = 1.0
        self.move_body_joint('body_roll_joint', -1.57, time_from_start)
        self.move_head_joint('head_pan_joint', 1.57, time_from_start)
        rospy.sleep(10)
        time_from_start = 0.5
        moving_cm = 0
        if self.from_base_to_shoulder_z_cm < object_z_cm:
            print "objectが肩の高さより高い場合"
            lift_cm = object_z_cm - self.from_base_to_shoulder_z_cm
            self.move_body_joint('body_lift_joint', lift_cm * 0.01, time_from_start)
            rospy.sleep(time_from_start)
            self.move_left_arm_joint('left_shoulder_flex_joint', -1.57, time_from_start)
            self.move_left_arm_joint('left_shoulder_roll_joint', 0.0, time_from_start)
            self.move_left_arm_joint('left_wrist_flex_joint', 0.0, time_from_start)
            moving_cm = math.sqrt(object_x_cm**2 + object_y_cm**2) - self.from_base_to_shoulder_x_cm - self.from_shoulder_to_wrist_x_cm
        else:
            print "objectが肩の高さより低い場合"
            from_shoulder_to_object_z = self.from_base_to_shoulder_z_cm - object_z_cm
            from_shoulder_to_object_x = math.sqrt(object_x_cm**2 + object_y_cm**2) - self.from_base_to_shoulder_x_cm
            left_shoulder_flex_joint_rad = -1.57 + math.atan2(from_shoulder_to_object_z, from_shoulder_to_object_x)
            left_wrist_flex_joint_rad = -math.atan2(from_shoulder_to_object_z, from_shoulder_to_object_x)
            rospy.sleep(time_from_start)
            self.move_left_arm_joint('left_wrist_flex_joint', left_wrist_flex_joint_rad, time_from_start)
            self.move_left_arm_joint('left_shoulder_flex_joint', left_shoulder_flex_joint_rad, time_from_start)
            print 'left_shoulder_flex_joint : ', left_shoulder_flex_joint_rad
            print 'left_wrist_flex_joint : ', left_wrist_flex_joint_rad
            moving_cm = math.sqrt(object_x_cm**2 + object_y_cm**2) - self.from_base_to_shoulder_x_cm - self.from_shoulder_to_wrist_x_cm * math.cos(left_wrist_flex_joint_rad)

        turning_deg = math.atan(object_y_cm / object_x_cm) * 57.2958
        str_turning_deg = 'T:' + str(turning_deg)
        self.move_wheel(str_turning_deg)
        rospy.sleep(2)
        str_moving_cm = 'S:' + str(moving_cm)
        self.move_wheel(str_moving_cm)

        return gripper_moveResponse(True)

    def jedge_right_hand_grasping(self, req_msg):
        right_hand_motor_joint_rad = 0
        for i in range(len(self.joint_info.name)):
            if self.joint_info.name[i] == 'right_hand_motor_joint':
                right_hand_motor_joint_rad = self.joint_info.position[i]
        if (right_hand_motor_joint_rad > 0.1):
            return grasping_jedgmentResponse(True)
        else:
            return grasping_jedgmentResponse(False)

    def jedge_left_hand_grasping(self, req_msg):
        left_hand_motor_joint_rad = 0
        for i in range(len(self.joint_info.name)):
            if self.joint_info.name[i] == 'left_hand_motor_joint':
                left_hand_motor_joint_rad = self.joint_info.position[i]
        if (left_hand_motor_joint_rad > 0.1):
            return grasping_jedgmentResponse(True)
        else:
            return grasping_jedgmentResponse(False)

    def open_and_close_right_gripper_server(self, req_msg):
        hand_motor_joint_rad = req_msg.rad
        time_from_start = 0.1
        self.move_right_arm_joint('right_hand_motor_joint', hand_motor_joint_rad, 0.3)
        rospy.sleep(time_from_start)
        return gripper_ctrlResponse(True)

    def open_and_close_left_gripper_server(self, req_msg):
        hand_motor_joint_rad = req_msg.rad
        time_from_start = 0.1
        self.move_left_arm_joint('left_hand_motor_joint', hand_motor_joint_rad, 0.3)
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
        elif motion_type == "CAMERA_PAN_LEFT":
            self.move_camera_pan_left()
        elif motion_type == "CAMERA_PAN_RIGHT":
            self.move_camera_pan_right()
        return robot_motionResponse(True)

    def check_publishers_connection(self, publisher):
        loop_rate_to_check_connection = rospy.Rate(1)
        while (publisher.get_num_connections() == 0 and not rospy.is_shutdown()):
            try:
                loop_rate_to_check_connection.sleep()
            except rospy.ROSInterruptException:
                pass

    def add_body_control_data_to_storage(self, joint_name, rad):
        self.body_control_data.joint_names.append(joint_name)
        self.body_control_data.points[0].positions.append(rad)
        self.body_control_data.points[0].velocities.append(0)
        self.body_control_data.points[0].accelerations.append(0)
        self.body_control_data.points[0].effort.append(0)

    def add_head_control_data_to_storage(self, joint_name, rad):
        self.head_control_data.joint_names.append(joint_name)
        self.head_control_data.points[0].positions.append(rad)
        self.head_control_data.points[0].velocities.append(0)
        self.head_control_data.points[0].accelerations.append(0)
        self.head_control_data.points[0].effort.append(0)

    def add_left_arm_control_data_to_storage(self, joint_name, rad):
        self.left_arm_control_data.joint_names.append(joint_name)
        self.left_arm_control_data.points[0].positions.append(rad)
        self.left_arm_control_data.points[0].velocities.append(0)
        self.left_arm_control_data.points[0].accelerations.append(0)
        self.left_arm_control_data.points[0].effort.append(0)

    def add_right_arm_control_data_to_storage(self, joint_name, rad):
        self.right_arm_control_data.joint_names.append(joint_name)
        self.right_arm_control_data.points[0].positions.append(rad)
        self.right_arm_control_data.points[0].velocities.append(0)
        self.right_arm_control_data.points[0].accelerations.append(0)
        self.right_arm_control_data.points[0].effort.append(0)

    def publish_body_control_data(self, time_from_start_sec):
        self.body_control_data.points[0].time_from_start = rospy.Duration(time_from_start_sec)
        self.check_publishers_connection(self.pub_body_control)
        self.pub_body_control.publish(self.body_control_data)
        self.body_control_data = JointTrajectory()
        self.body_control_data.points = [JointTrajectoryPoint()]

    def publish_head_control_data(self, time_from_start_sec):
        self.head_control_data.points[0].time_from_start = rospy.Duration(time_from_start_sec)
        self.check_publishers_connection(self.pub_head_control)
        self.pub_head_control.publish(self.head_control_data)
        self.head_control_data = JointTrajectory()
        self.head_control_data.points = [JointTrajectoryPoint()]

    def publish_left_arm_control_data(self, time_from_start_sec):
        self.left_arm_control_data.points[0].time_from_start = rospy.Duration(time_from_start_sec)
        self.check_publishers_connection(self.pub_left_arm_control)
        self.pub_left_arm_control.publish(self.left_arm_control_data)
        self.left_arm_control_data = JointTrajectory()
        self.left_arm_control_data.points = [JointTrajectoryPoint()]

    def publish_right_arm_control_data(self, time_from_start_sec):
        self.right_arm_control_data.points[0].time_from_start = rospy.Duration(time_from_start_sec)
        self.check_publishers_connection(self.pub_right_arm_control)
        self.pub_right_arm_control.publish(self.right_arm_control_data)
        self.right_arm_control_data = JointTrajectory()
        self.right_arm_control_data.points = [JointTrajectoryPoint()]

    def move_wheel(self, str_distance):
        rospy.wait_for_service('/robot_ctrl/odom_base_ctrl')
        try:
            wheel_ctrl_service = rospy.ServiceProxy('/robot_ctrl/odom_base_ctrl', odom_base)
            res = wheel_ctrl_service(str_distance)
            return res.res_str
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e

    def move_to_initial_pose(self):
        time_from_start_sec = 0.3
        self.add_body_control_data_to_storage('body_lift_joint', 0.0)
        self.add_body_control_data_to_storage('body_roll_joint', 0)
        self.add_head_control_data_to_storage('head_tilt_joint', 0.3)
        self.add_head_control_data_to_storage('head_pan_joint', 0)
        self.add_right_arm_control_data_to_storage('right_shoulder_roll_joint', 0)
        self.add_right_arm_control_data_to_storage('right_shoulder_flex_joint', -0.82)
        self.add_right_arm_control_data_to_storage('right_wrist_flex_joint', 0)
        self.add_right_arm_control_data_to_storage('right_hand_motor_joint', 0)
        self.add_left_arm_control_data_to_storage('left_shoulder_roll_joint', 0)
        self.add_left_arm_control_data_to_storage('left_shoulder_flex_joint', -0.82)
        self.add_left_arm_control_data_to_storage('left_wrist_flex_joint', 0)
        self.add_left_arm_control_data_to_storage('left_hand_motor_joint', 0)
        self.publish_body_control_data(time_from_start_sec)
        self.publish_head_control_data(time_from_start_sec)
        self.publish_left_arm_control_data(time_from_start_sec)
        self.publish_right_arm_control_data(time_from_start_sec)
        rospy.sleep(time_from_start_sec)

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
        self.move_head_joint('head_tilt_joint', 0.28, 0.1)
        rospy.sleep(time_from_start)

    def move_camera_pan_left(self):
        self.move_head_joint('head_pan_joint', 1.57, 4.0)

    def move_camera_pan_right(self):
        self.move_head_joint('head_pan_joint', -1.57, 8.0)


if __name__ == "__main__":
    rospy.init_node("joint_controller")
    rospy.sleep(1)
    jc = JointController()
    jc.move_to_initial_pose()
    rospy.spin()
