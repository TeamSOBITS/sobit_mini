# sobit_mini_library
Library to control SOBIT (based on Turtlebot)

---

## How to Install
```bash:
$ sudo apt update 
$ sudo apt install ros-melodic-pybind11-catkin -y 
or $ sudo apt install ros-kinetic-pybind11-catkin -y 
$ cm
```

---

## SobitMiniController
sobit miniを動かすクラス
### Joint
* sobit miniのジョイント名とその定数名(Enum : Joint)
    ```bash
    * "head_pan_joint" : HEAD_PAN_JOINT
    * "head_tilt_joint" : HEAD_TILT_JOINT 
    * "body_roll_joint" : BODY_ROLL_JOINT                           
    * "right_shoulder_roll_joint" : RIGHT_SHOULDER_ROLL_JOINT
    * "right_shoulder_flex_joint" : RIGHT_SHOULDER_FLEX_JOINT
    * "right_elbow_roll_joint" : RIGHT_ELBOW_ROLL_JOINT
    * "right_hand_motor_joint" : RIGHT_HAND_MOTOR_JOINT
    * "left_shoulder_roll_joint" : LEFT_SHOULDER_ROLL_JOINT
    * "left_shoulder_flex_joint" : LEFT_SHOULDER_FLEX_JOINT
    * "left_elbow_roll_joint" : LEFT_ELBOW_ROLL_JOINT
    * "left_hand_motor_joint" : LEFT_HAND_MOTOR_JOINT
    '''

### Functions
#### WheelController
1.  controlWheelLinear() :   直線移動
    ```bash
    bool sobit::SobitMiniController::controlWheelLinear( 
        const double distance           :   移動距離[m]
    )
    ```  
2.  controlWheelRotateRad() :   回転移動
    ```bash
    bool sobit::SobitMiniController::controlWheelRotateRad( 
        const double angle_rad           :   回転角度[rad]
    )
    ```  
3.  controlWheelRotateDeg() :   回転移動
    ```bash
    bool sobit::SobitMiniController::controlWheelRotateDeg( 
        const double angle_rad           :   回転角度[deg]
    )
    ```  
    
#### JointController
1.  moveJoint() :   1つのジョイントを動かす関数（ジョイントの指定は定数名(Enum)）
    ```bash
    bool sobit::SobitMiniController::moveJoint (
        const Joint joint_num,          :   ジョイント番号
        const double rad,               :   移動角度
        const double sec,               :   移動時間
        bool is_sleep = true            :   移動中にスリープ(待機)を入れるかどうか
    )
    ```  
2.  moveHeadPanTilt()   :   xtionのパンチルトを任意の角度に動かす
    ```bash
    bool sobit::SobitMiniController::moveHeadPanTilt (
        const double pan_rad,           :   パン角度
        const double tilt_rad,          :   チルト角度
        const double sec,               :   移動時間
        bool is_sleep = true            :   移動中にスリープ(待機)を入れるかどうか
    )
    ```  
3.  moveRightArm()   :   右のアームを任意の角度に動かす
    ```bash
    bool sobit::SobitMiniController::moveRightArm ( 
        const double arm_roll,           
        const double arm_flex,             
        const double elbow_flex,                   
        const double hand_motor,            
    )
    ```  
4.  moveLeftrm()   :   左のアームを任意の角度に動かす
    ```bash
    bool sobit::SobitMiniController::moveLeftArm ( 
        const double arm_roll,           
        const double arm_flex,             
        const double elbow_flex,                   
        const double hand_motor,            
    )
    ```  
4.  movePose()   :   予め設定したポーズに動かす
    ```bash
    bool sobit::SobitMiniController::movePose( 
        const std::string &pose_name 
    )
    ```  

    * ポーズのロード方法：sobit_mini_library/launch/load_sobit_mini_pose.launchでポーズをros paramで設定する
    ```bash:
    $ roslaunch sobit_mini_library load_sobit_mini_pose.launch
    ```

    * ポーズの設定方法：sobit_mini_library/config/sobit_mini_pose.yamlでポーズを設定する
    ```bash
    mini_pose:
        - { 
            pose_name: "initial_pose",
            head_pan_joint: 0.0,
            head_tilt_joint: 0.0,     
            body_roll_joint: 0.0,                             
            right_shoulder_roll_joint: 0.0,
            right_shoulder_flex_joint: -0.82,
            right_elbow_roll_joint: 0.0,
            right_hand_motor_joint: 0.0,
            left_shoulder_roll_joint: 0.0,
            left_shoulder_flex_joint: -0.82,
            left_elbow_roll_joint: 0.0,
            left_hand_motor_joint : 0.0
        }

        - { 
            pose_name: "attention_pose",
            head_pan_joint: 0.0,
            head_tilt_joint: 0.0,       
            body_roll_joint: 0.0,                           
            right_shoulder_roll_joint: 0.0,
            right_shoulder_flex_joint: -1.30,
            right_elbow_roll_joint: 0.0,
            right_hand_motor_joint: 0.0,
            left_shoulder_roll_joint: 0.0,
            left_shoulder_flex_joint: -1.30,
            left_elbow_roll_joint: 0.0,
            left_hand_motor_joint : 0.0
        }

        - { 
            pose_name: "receive_pose", 
            head_pan_joint: -0.2,
            head_tilt_joint: 0.0,     
            body_roll_joint: 0.0,                            
            right_shoulder_roll_joint: 0.0,
            right_shoulder_flex_joint: -1.30,
            right_elbow_roll_joint: 1.57,
            right_hand_motor_joint: 0.0,
            left_shoulder_roll_joint: 0.0,
            left_shoulder_flex_joint: -1.30,
            left_elbow_roll_joint: 0.0,
            left_hand_motor_joint : 0.0
        }
    ```  
---

## How to use
### C++

```bash:
#include <sobit_mini_library/sobit_mini_controller.hpp>
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "sobit_turtlebot_controller_test");
    sobit::SobitMiniController sobit_mini_ctr;
    ros::Rate loop_rate(1.0);
    double pan_ang = 0.8;
    while ( ros::ok() ) {
        pan_ang *= -1.0;
        sobit_mini_ctr.moveJoint( sobit::Joint::HEAD_PAN_JOINT, pan_ang, 0.8, false );
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}
```

### Python
```bash:
#!/usr/bin/env python
import rospy
from sobit_mini_module import SobitMiniController
from sobit_mini_module import Joint
import sys

def test():
    rospy.init_node('test')
    r = rospy.Rate(1) # 10hz
    ang = 0.8
    args = sys.argv
    edu_ctr = SobitMiniController(args[0]) # args[0] : C++上でros::init()を行うための引数
    while not rospy.is_shutdown():
        ang = -1.0 * ang
        edu_ctr.moveJoint( Joint.HEAD_PAN_JOINT, ang, 0.8, false )
        r.sleep()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: pass
    
```

---