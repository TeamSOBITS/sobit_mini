# SOBIT MINI

<!--目次-->
<details>
    <summary>目次</summary>
    <ol>
        <li>
            <a href="概要">概要</a>
        </li>
    </ol>
</details>

<!--レポジトリの概要-->
## 概要
![](sobit_mini/img/sobit_mini.png)

SOBITSが開発した双腕型モバイルマニピュレータ（SOBIT MINI）を動かすためのライブラリです．

> [!WARNING]
> 初心者の場合，実機のロボットを扱う際に，先輩方に付き添ってもらいながらロボットを動かしましょう．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- セットアップ -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明します．

### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| Python | 3.0~ |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBIT Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)に参照してください．

<!-- - OS: Ubuntu 20.04 
- ROS distribution: noetic Kame -->

### インストール方法

1. ROSの`src`フォルダに移動します．
   ```sh
   $ roscd
   # もしくは，"cd ~/catkin_ws/"へ移動．
   $ cd src/
   ```
2. 本レポジトリをcloneします．
   ```sh
   $ git clone https://github.com/TeamSOBITS/sobit_mini
   ```
3. レポジトリの中へ移動します．
   ```sh
   $ cd sobit_mini/
   ```
4. 依存パッケージをインストールします．
   ```sh
   $ bash install.sh
   ```
5. パッケージをコンパイルします．
   ```sh
   $ roscd
   # もしくは，"cd ~/catkin_ws/"へ移動．
   $ catkin_make
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法

1. [minimal.launch](sobit_mini_bringup/launch/minimal.launch)というlaunchファイルを起動します．
   ```sh
   $ roslaunch sobit_mini_bringup minimal.launch
   ```
2. [任意] デモプログラムを実行してみましょう．
   ```sh
   $ rosrun sobit_mini_library test_controll_wheel.py
   ```

> [!NOTE]
> SOBIT MINIの動作方法になれるため，[example](sobit_mini_library/example/)フォルダを確認し，それぞれのサンプルファイルから動作関数を学びましょう．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### Rvizの可視化
実機を動かす前段階で，Rviz上でSOBIT MINIを可視化し，ロボットの構成を表示することができます．

```sh
$ roslaunch sobit_mini_description display.launch
```

正常に動作した場合は，次のようにRvizが表示されます．

![SOBIT MINI Display with Rviz](sobit_mini/img/sobit_mini_display.png)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## ソフトウェア
<details>
<summary>SOBIT MINIと関わるソフトの情報まとめ</summary>


### ジョイントコントローラ
SOBIT_MINIのパンチルト機構とマニピュレータを動かすための情報まとめです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### 動作関数
1.  `moveToPose()` : 決められたポーズに動かします．
   ```cpp
   bool moveToPose( const std::string &pose_name //ポーズ名
   );
   ```

> [!NOTE]
> 既存のポーズは[sobit_mini_pose.yaml](sobit_mini_library/config/sobit_mini_pose.yaml)に確認できます．

2. `moveHeadPanTilt` : パンチルト機構を任意の角度に動かします．
   ```cpp
   bool moveHeadPanTilt(
      const double pan_rad,         // 回転角度 [rad]
      const double tilt_rad,        // 回転角度 [rad]
      const double sec,             // 回転時間 [s]
      bool is_sleep                 // 回転後に待機するかどうか
   )
   ```

3. `moveRightArm` : 右腕のジョイントを任意の角度に動かします．
   ```cpp
   bool moveRightArm(
      const double shoulder_roll,   // 回転角度 [rad]
      const double shoulder_pan,    // 回転角度 [rad]
      const double elbow_tilt,      // 回転角度 [rad]
      const double wrist_tilt,      // 回転角度 [rad]
      const double hand_motor,      // 回転角度 [rad]
      const double sec,             // 回転時間 [s]
      bool is_sleep                 // 回転後に待機するかどうか
   )
   ```

4. `moveLeftArm` : 右腕のジョイントを任意の角度に動かします．
   ```cpp
   bool moveLeftArm(
      const double shoulder_roll,   // 回転角度 [rad]
      const double shoulder_pan,    // 回転角度 [rad]
      const double elbow_tilt,      // 回転角度 [rad]
      const double wrist_tilt,      // 回転角度 [rad]
      const double hand_motor,      // 回転角度 [rad]
      const double sec,             // 回転時間 [s]
      bool is_sleep                 // 回転後に待機するかどうか
   )
   ```

5. `moveJoint` : 指定されたジョイントを任意の角度に動かします．
   ```cpp
   bool moveJoint(
      const Joint joint_num,  // ジョイント名 (定数名)
      const double rad,       // 回転角度 [rad]
      const double sec,       // 回転時間 [s]
      bool is_sleep           // 回転後に待機するかどうか
   )
   ```

6. `moveAllJoint` : 全てのジョイントを任意の角度に動かします．
   ```cpp
   bool moveAllJoint(
      const double l_arm_shoulder_roll_joint,   // 回転角度 [rad]
      const double l_arm_shoulder_pan_joint,    // 回転角度 [rad]
      const double l_arm_elbow_tilt_joint,      // 回転角度 [rad]
      const double l_hand_joint,                // 回転角度 [rad]
      const double r_arm_shoulder_roll_joint,   // 回転角度 [rad]
      const double r_arm_shoulder_pan_joint,    // 回転角度 [rad]
      const double r_arm_elbow_tilt_joint,      // 回転角度 [rad]
      const double r_arm_wrist_tilt_joint,      // 回転角度 [rad]
      const double r_hand_joint,                // 回転角度 [rad]
      const double body_roll_joint,             // 回転角度 [rad]
      const double head_pan_joint,              // 回転角度 [rad]
      const double head_tilt_joint,             // 回転角度 [rad]
      const double sec,                         // 回転時間 [s]
      bool is_sleep                             // 回転後に待機するかどうか
   )
   ```

7. `moveGripperToTargetCoord` : ハンドをxyz座標に動かします（把持モード）．
   ```cpp
   bool moveGripperToTargetCoord(
      const int arm_mode,                 //使用するアーム(arm_mode=0:左腕,arm_mode=1:左腕)
      const double goal_position_x,       //把持目的地のx [m]
      const double goal_position_y,       //把持目的地のy [m]
      const double goal_position_z,       //把持目的地のz [m]
      const double diff_goal_position_x,  // xyz座標のx軸をシフトする [m]
      const double diff_goal_position_y,  // xyz座標のy軸をシフトする [m]
      const double diff_goal_position_z   // xyz座標のz軸をシフトする [m]
   )
   ```

   8. `moveGripperToTargetTF` : ハンドをtf名に動かします（把持モード）．
   ```cpp
   bool moveGripperToTargetTF(
      const int arm_mode,                    //使用するアーム(arm_mode=0:左腕,arm_mode=1:左腕)
      const std::string &goal_position_name, //把持目的tf名
      const double diff_goal_position_x,     // xyz座標のx軸をシフトする [m]
      const double diff_goal_position_y,     // xyz座標のy軸をシフトする [m]
      const double diff_goal_position_z      // xyz座標のz軸をシフトする [m]
   )
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>




まず，以下のコマンドを入力して，SOBIT MINIを動かすための環境設定を行います．
この設定は，初回のみに行う作業ですので，1度行ったことのある人は飛ばしてください．

※ 開発するPCで，SOBIT EDUやSOBIT PROを動かしたことがある場合も，この作業は必要ありません．

```bash:
$ cd sobit_mini
$ bash sobit_setup.sh
```

以下のコマンドを入力することで，SOBIT MINIを起動することができます．
これにより，SOBIT MINIのモータやRGB-Dカメラ，測域センサ(Lidar)などのデバイスが起動します．
また，それと同時にRvizも起動します．

:warning: ロボットをコンテナで動かす場合，動かしたいデバイスをホストPCと接続してから，コンテナを立ち上げてください．
コンテナを立ち上げてからデバイスとの接続を行う場合，ロボットが動かない場合があります．

```bash:
$ roslaunch sobit_mini_bringup minimal.launch
```
