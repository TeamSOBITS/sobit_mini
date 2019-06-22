#!/usr/bin/env python
# coding: utf-8

import rospy
import time
import math
import sympy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from wheel_test.srv import wheel_control
from tf.transformations import euler_from_quaternion

odometry_value = Odometry() # 初期位置
Kp = Float64()
Kv = Float64()
Ki = Float64()

# サービスでの要求を読み取る
def request(req):
    global odometry_value

    t1 = time.time() # 処理前の時刻
    speed = Twist()
    rate = rospy.Rate(10)
    xt = 0.0

    # 初期の位置を保存する
    initial_value = odometry_value

    if "S" in req.vector:
        rospy.loginfo("order_Straight")

        order_value = read_value(req) # 移動距離を読み取る

        while xt < abs(order_value):

            result = pid_calculation(abs(order_value), xt, t1, req.max_speed) # PID制御の計算を行う

            if order_value < 0:
                speed.linear.x = -result
            else:
                speed.linear.x = result

            pub.publish(speed) # トピックを用いて指定の速さを送る

            x_diff = odometry_value.pose.pose.position.x - initial_value.pose.pose.position.x
            y_diff = odometry_value.pose.pose.position.y - initial_value.pose.pose.position.y
            xt = math.sqrt(x_diff ** 2 + y_diff ** 2) # ユークリッド距離の計算

            #rospy.loginfo("%s %s", xt, order_value)
            rate.sleep()

    elif "T" in req.vector:
        rospy.loginfo("order_Trun")

        order_value = math.radians(read_value(req)) # 度数法での回転角を読み取り弧度法に変換する

        while xt < abs(order_value):

            result = pid_calculation(abs(order_value), xt, t1, req.max_speed) #PID制御の計算を行う

            if order_value < 0:
                speed.angular.z = -result
            else:
                speed.angular.z = result

            pub.publish(speed) # トピックを用いて指定の速さを送る

            odometry_euler = tf.transformations.euler_from_quaternion((odometry_value.pose.pose.orientation.x,odometry_value.pose.pose.orientation.y,odometry_value.pose.pose.orientation.z,odometry_value.pose.pose.orientation.w))
            initial_euler = tf.transformations.euler_from_quaternion((initial_value.pose.pose.orientation.x,initial_value.pose.pose.orientation.y,initial_value.pose.pose.orientation.z,initial_value.pose.pose.orientation.w))
            xt = abs(odometry_euler[2] - initial_euler[2]) # ユークリッド距離の計算

            #rospy.loginfo("%s %s", xt, order_value)
            rate.sleep()

    xt = 0.0
    return 'finished'

# 値を取り出す
def read_value(req):
    value_str = req.vector[2:len(req.vector)]
    value = float(value_str)
    return value

# PID制御の計算を行う
def pid_calculation(xd, xt, t1, max_speed):
    global Kp
    global Kv
    global Ki

    x = sympy.Symbol('x') # 変数の定義

    t2 = time.time() # 処理後の時刻
    elapsed_time = t2-t1 # 経過時間

    ft = Kp * (xd - xt) - Kv * sympy.diff(x.subs(x, xt), x) + Ki * sympy.integrate(xd - x.subs(x, xt), (x, 0, elapsed_time)) #PID制御

    if max_speed == 0.0:
        return ft
    elif max_speed < ft:
        return max_speed
    else:
        return ft

# 車輪のオドメトリを返す
def odometory_save(odometry):
    global odometry_value

    odometry_value = odometry

# メイン
if __name__ == '__main__':
    rospy.init_node('wheel_control')
    global Kp
    global Kv
    global Ki

    sub = rospy.Subscriber('/odom', Odometry, odometory_save)
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
    rospy.Service('wheel_control', wheel_control, request)

    # パラメータの設定
    Kp = rospy.get_param("/proportional_control", 0.1)
    Kv = rospy.get_param("/derivation_control", 0.4)
    Ki = rospy.get_param("/integral_control", 0.1)

    print "Ready to serve"
    rospy.spin()
