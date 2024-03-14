<a name="readme-top"></a>

[JA](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBIT MINI

<!--目次-->
<details>
   <summary>目次</summary>
   <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li>
    <a href="#実行操作方法">実行・操作方法</a>
      <ul>
        <li><a href="#Rviz上の可視化">Rviz上の可視化</a></li>
      </ul>
    </li>
    <li>
    <a href="#ソフトウェア">ソフトウェア</a>
      <ul>
        <li><a href="#ジョイントコントローラ">ジョイントコントローラ</a></li>
        <li><a href="#ホイールコントローラ">ホイールコントローラ</a></li>
      </ul>
    </li>
    <li>
    <a href="#ハードウェア">ハードウェア</a>
      <ul>
        <li><a href="#パーツのダウンロード方法">パーツのダウンロード方法</a></li>
        <li><a href="#電子回路図">電子回路図</a></li>
        <li><a href="#ロボットの組み立て">ロボットの組み立て</a></li>
        <li><a href="#ロボットの特徴">ロボットの特徴</a></li>
        <li><a href="#部品リストBOM">部品リスト（BOM）</a></li>
      </ul>
    </li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#参考文献">参考文献</a></li>
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
   $ rosrun sobit_mini_library test_control_wheel.py
   ```

> [!NOTE]
> SOBIT MINIの動作方法になれるため，[example](sobit_mini_library/example/)フォルダを確認し，それぞれのサンプルファイルから動作関数を学びましょう．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### Rviz上の可視化
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
SOBIT MINIのパンチルト機構とマニピュレータを動かすための情報まとめです．

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
      const double hand_rad,              //ハンドの開閉角度の調整
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
      const double hand_rad,                 //ハンドの開閉角度の調整
      const double diff_goal_position_x,     // xyz座標のx軸をシフトする [m]
      const double diff_goal_position_y,     // xyz座標のy軸をシフトする [m]
      const double diff_goal_position_z      // xyz座標のz軸をシフトする [m]
   )
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

#### ジョイント名

SOBIT MINIのジョイント名とその定数名は以下の通りです．

| ジョイント番号 | ジョイント名 | ジョイント定数名 |
| :---: | --- | --- |
| 0 | l_arm_shoulder_roll_joint | L_ARM_SHOULDER_ROLL_JOINT |
| 1 | l_arm_shoulder_pan_joint | L_ARM_SHOULDER_PAN_JOINT |
| 2 | l_arm_elbow_tilt_joint | L_ARM_ELBOW_TILT_JOINT |
| 3 | l_arm_wrist_tilt_joint | L_ARM_WRIST_TILT_JOINT |
| 4 | l_hand_joint | L_HAND_JOINT |
| 5 | r_arm_shoulder_roll_joint | R_ARM_SHOULDER_ROLL_JOINT |
| 6 | r_arm_shoulder_pan_joint | R_ARM_SHOULDER_PAN_JOINT |
| 7 | r_arm_elbow_tilt_joint | R_ARM_ELBOW_ROLL_JOINT |
| 8 | r_arm_wrist_tilt_joint | R_ARM_WRIST_TILT_JOINT |
| 9 | r_hand_joint | R_HAND_JOINT |
| 10 | body_roll_joint | BODY_ROLL_JOINT |
| 11 | head_pan_joint | HEAD_PAN_JOINT |
| 12 | head_tilt_joint | HEAD_TILT_JOINT |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### ポーズの設定方法

[sobit_mini_pose.yaml](sobit_mini_library/config/sobit_mini_pose.yaml)というファイルでポーズの追加・編集ができます．以下のようなフォーマットになります．

```yaml
mini_pose:
    - { 
        pose_name: "pose_name",
        l_arm_shoulder_roll_joint: 0.0,
        l_arm_shoulder_pan_joint: -1.25,
        l_arm_elbow_tilt_joint: 0.0,
        l_arm_wrist_tilt_joint: 0.0,
        l_hand_joint: 0.0,
        r_arm_shoulder_roll_joint: 0.0,
        r_arm_shoulder_pan_joint: -1.25,
        r_arm_elbow_tilt_joint: 0.0,
        r_arm_wrist_tilt_joint: 0.0,
        r_hand_joint: 0.0,
        body_roll_joint: 0.0,
        head_pan_joint: 0.0,
        head_tilt_joint: 0.0
    }
```

### ホイールコントローラ

SOBIT MINIの移動機構部を動かすための情報まとめです．


#### 動作関数

1. `controlWheelLinear()` : 並進（前進・後進）に移動させます．
   ```cpp
   bool controlWheelLinear(const double distance      //x方向への直進移動距離
   )
   ```

2. `controlWheelRotateRad()` : 回転運動を行う（弧度法：Radian）
   ```cpp
   bool controlWheelRotateRad(const double angle_rad  // 中心回転角度 [rad]
   )
   ```

3. `controlWheelRotateDeg()` : 回転運動を行う（度数法：Degree）
   ```cpp
   bool controlWheelRotateDeg(const double angle_deg  // 中心回転角度 (deg)
   )
   ```

</details>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

## ハードウェア

SOBIT MINIはオープンソースハードウェアとして[Onshape](https://cad.onshape.com/documents/8875b6e7a5f6f87b4f951969/w/d265c3a1708d61e2a005595d/e/00fdacbdb703dc27e5e0d3f8)にて公開しております．

![SOBIT MINI in OnShape](sobit_mini/img/sobit_mini_onshape.png)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<details>
<summary>ハードウェアの詳細についてはこちらを確認してください．</summary>

### パーツのダウンロード方法

1. Onshapeにアクセスしましょう．

> [!NOTE]
> ファイルをダウンロードするために，`OnShape`のアカウントを作成する必要がありません．ただし，本ドキュメント全体をコピーする場合，アカウントの作成を推奨します．

2. `Instance`の中にパーツを右クリックで選択します．
3. 一覧が表示され，`Export`ボタンを押してください．
4. 表示されたウィンドウの中に，`Format`という項目があります．`STEP`を選択してください．
5. 最後に，青色の`Export`ボタンを押してダウンロードが開始されます．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### 電子回路図

TBD

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### ロボットの組み立て

TBD

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### ロボットの特徴

| 項目 | 詳細 |
| --- | --- |
| 最大直進速度 | 0.65[m/s] |
| 最大回転速度 | 3.1415[rad/s] |
| 最大ペイロード | 0.35[kg] |
| サイズ (長さx幅x高さ) | 512x418x1122[mm] |
| 重量 | 11.6[kg] |
| リモートコントローラ | PS3/PS4 |
| LiDAR | UST-10LX |
| RGB-D | Intel Realsense D435F |
| スピーカー | モノラルスピーカー |
| マイク | コンデンサーマイク |
| アクチュエータ (アーム) | 2 x XM540-W150, 9 x XM430-W320 |
| 移動機構 | TurtleBot2 |
| 電源 | 2 x Makita 6.0Ah 18V |
| PC接続 | USB |


### 部品リスト（BOM）

| 部品 | 型番 | 個数 | 購入先 |
| --- | --- | --- | --- |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |
| --- | --- | 1 | [link]() |


</details>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

- [x] exampleファイルの修正
- [x] OSS
    - [x] ドキュメンテーションの充実
    - [x] コーディングスタイルの統一

現時点のバッグや新規機能の依頼を確認するために[Issueページ][license-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more NOTErmation.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->

<!-- 参考文献 -->
## 参考文献

* [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
* [ROS Noetic](http://wiki.ros.org/noetic)
* [ROS Control](http://wiki.ros.org/ros_control)


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobit_mini.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobit_mini/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobit_mini.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobit_mini/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobit_mini.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobit_mini/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobit_mini.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobit_mini/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobit_mini.svg?style=for-the-badge
[license-url]: https://github.com/TeamSOBITS/sobit_mini/issues



<!-- まず，以下のコマンドを入力して，SOBIT MINIを動かすための環境設定を行います．
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
``` -->
