<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

> [!WARNING]
> 本ロボット，及び本リポジトリはサポートされて間もないため，今後も頻繁に大きく改良される可能性があります．

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBIT LIGHT

<!-- 目次 -->
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
        <li><a href="#シミュレータの実行方法">シミュレータの実行方法</a></li>
      </ul>
    </li>
    <li>
    　<a href="#ソフトウェア">ソフトウェア</a>
      <ul>
        <li><a href="#ジョイントコントローラ">ジョイントコントローラ</a></li>
        <li><a href="#ホイルコントローラ">ホイルコントローラ</a></li>
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



<!-- レポジトリの概要 -->
## 概要

![SOBIT LIGHT](sobit_light/docs/img/sobit_light.png)

Preferred Robotics(c)が開発した[カチャカ](https://kachaka.life/home/)を用いたSOBITS自作のモバイルマニピュレータを動かすためのライブラリです．

> [!WARNING]
> 初心者の場合，実機のロボットを扱う際に，先輩方に付き添ってもらいながらロボットを動かしましょう．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill |
| Python | 3.10 |
| Docker | latest |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)に参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

**SOBIT LIGHTを使用するローカル環境，またはコンテナ内でのセットアップ内容**
1. ROSの`src`フォルダに移動します．
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. 本レポジトリをcloneします．
    ```sh
    $ git clone https://github.com/TeamSOBITS/sobit_light
    ```

3. レポジトリの中へ移動します．
    ```sh
    $ cd sobit_light/
    ```

4. 依存パッケージをインストールします．
    ```sh
    $ bash install.sh
    ```

5. パッケージをコンパイルします．
    ```sh
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install
    $ source ~/colcon_ws/install/setup.sh
    ```

**ローカル環境でのセットアップ内容**
1. Kachaka APIのリポジトリをcloneします．
    ```sh
    $ cd ~/
    $ git clone https://github.com/TeamSOBITS/kachaka-api.git
    ```

2. 最新のDockerイメージをビルドします．
    ```sh
    $ cd kachaka-api/
    $ docker buildx build -t kachaka-api --target kachaka-grpc-ros2-bridge -f Dockerfile.ros2 . --build-arg BASE_ARCH=x86_64 --load
    ```

3. `ROS_DOMAIN_ID`を設定します．一例として，`10`とします．
    ```sh
    $ echo 'export ROS_DOMAIN_ID=10' >> ~/.bashrc
    $ source ~/.bashrc
    ```

> [!IMPORTANT]
> データ通信のため，ローカル環境以外(Docker等)でROSのワークスペースを使用している場合は，`ROS_DOMAIN_ID`の値を統一させる必要があることを忘れないでください．

4. KachakaのIPアドレスを確認します．
    1. One way is to ask Kachaka by saying, "Hey Kachaka, what's your IP address?"    
        Kachaka will then read out the IP address.
    2. Another way is to open the `Settings` tab in the Kachaka app, tap on `App Information` in the `Settings & Information` category, and check the `IP Address` field in the `Kachaka` category.

5. KachakaとのROS Bridgeを簡単に立ち上げられるようにするために，`alias`を設定します．
    ```sh
    $ echo 'alias kachaka="bash ~/kachaka-api/tools/ros2_bridge/start_bridge.sh"' >> ~/.bashrc
    $ source ~/.bashrc
    ```

<!-- > [!NOTE]
> ここで作成したコンテナに関して，もしカチャカのIPアドレスが変わった場合は一度Dockerコンテナを消して1から行ってください． -->

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法

1. [ローカル環境] KachakaとのROS BridgeのDockerコンテナを立ち上げます．
    ```
    $ kachaka <カチャカのIPアドレス> sobit_light no
    ```
> [!NOTE]
> `sobit_light`を書くことによって，ロボットの`namespace`を設定しています．また，`no`では，Kachaka側のrobot_descriptionの発行を停止します．詳細については，[Dockerを使ったros2_bridgeの起動](https://github.com/TeamSOBITS/kachaka-api/blob/main/docs/ROS2.md#%E3%83%96%E3%83%AA%E3%83%83%E3%82%B8%E3%81%AE%E8%B5%B7%E5%8B%95)を確認してください．

> [!WARNING]
> KachakaのIPが変わる可能性がありますので，ご注意ください．

2. SOBIT LIGHTをインストールしている環境内で[real_minimal.launch](sobit_light_bringup/launch/real_minimal.launch.py)というlaunchファイルを実行します．
   ```sh
   $ ros2 launch sobit_light_bringup real_minimal.launch.py
   ```

3. ロボットが立ち上がらない・Kachakaとの通信ができていない場合は，次の項目を確認してください．

    - 緊急停止ボタンが押下されていないか
    - バッテリが十分に充電されているか 
    - USB hubがパソコンと接続されているか
    - [TODO] Dynamixel Dongleの名前は`/dev/ttyUSB0`なのか
    - - 確認するために`$ ls /dev`を書いて，`/dev/ttyUSB1`が表示される場合，[controllers.urdf.xacro](sobit_light_description/urdf/controllers.urdf.xacro)の`usb_port`を更新してください．
    - Kachaka IPが正しいか
    - `ROS_DOMAIN_ID`がカチャカ側と開発環境側と同じか

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### Rviz2上の可視化

実機を動かす前段階として，Rviz2上でSOBIT LIGHTを可視化し，ロボットの構成を表示することができます．

```sh
$ ros2 launch sobit_light_description display.launch.py
```

正常に動作した場合は，次のようなRviz画面が表示されます．
![SOBIT LIGHT Display with Rviz](sobit_light/docs/img/sobit_light_rviz.png)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>

### シミュレータの実行方法

SOBIT LIGHTにはGazebo Fortressのシミュレーション環境が用意されておりますので，実機がなくても，動作確認が可能です．

```sh
$ ros2 launch sobit_light_bringup gz_minimal.launch.py
```

正常に動作した場合は，次のようなGazeboの画面が表示されます．
![SOBIT LIGHT Gazebo Fortress](sobit_light/docs/img/sobit_light_gz_sim.png)

> [!WARNING]
> 実機と同じようなセンサも搭載されていますので，パソコンによって処理が重くなる可能性がありますので，必要なセンサだけを[gz_minimal.launch.py](sobit_light_bringup/launch/gz_minimal.launch.py)で選択してください．

```python
'enable_gz_front_cam_color' : 'True',
'enable_gz_back_cam_color' : 'True',
'enable_gz_head_cam_color' : 'True',
'enable_gz_head_cam_depth' : 'True',
'enable_gz_hand_cam_color' : 'True',
'enable_gz_hand_cam_depth' : 'True',
'enable_gz_lidar' : 'True',
'enable_gz_imu' : 'True',
```

また，複数のSOBIT LIGHTを同じシミュレーション環境でも出現できます．
そのために，[gz_minimal.launch.py](sobit_light_bringup/launch/gz_minimal.launch.py)でロボットの数に合わせて`gz_robot.launch.py`が実行されるようにその設定を加えてください．

`robot_name`はロボット間で異なる値を持つ必要があります．
さらに，`robot_coords_x`，`robot_coords_y`，および`robot_coords_z`でロボットの出現座標を変更できます．

一例はこちらとなります．

```python
...
# Launch Robot No. 1
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('sobit_light_bringup'),
            'launch',
            'robot.launch.py'
        ])
    ]),
    launch_arguments={
        'robot_name': 'sobit_light_1',
        'robot_coords_x': '0', # x 
        'robot_coords_y': '0', # y
        'robot_coords_Y': '0', # yaw
        ...
    }.items()
),
# Launch Robot No. 2
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('sobit_light_bringup'),
            'launch',
            'gz_robot.launch.py'
        ])
    ]),
    launch_arguments={
        'robot_name': 'sobit_light_2',
        'robot_coords_x': '0', # x 
        'robot_coords_y': '2', # y
        'robot_coords_Y': '0', # yaw
        ...
    }.items()
),
...
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## ソフトウェア

<details>
<summary>SOBIT LIGHTと関わるソフトの情報まとめ</summary>


### ジョイントコントローラ

SOBIT LIGHTのパンチルト機構とマニピュレータを動かすための情報まとめです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### アクション

1.  `move_to_pose` : 決められたポーズに動かします．
    ```yaml
    # MoveToPose.action
    # Goal
    string pose_name                                # 事前に定義したポーズ名
    builtin_interfaces/Duration time_allowance      # 制限時間
    ---
    # Result
    bool success                                    # 成功/失敗
    string message                                  # 結果メッセージ
    builtin_interfaces/Duration total_elapsed_time  # かかった時間
    ---
    # Feedback
    string[] current_joint_names                    # 現在の稼働関節名リスト
    float32[] current_joint_rad                     # 現在の稼働関節角度リスト
    # float32[] current_joint_vel                   # 現在の稼働関節の速度リスト
    builtin_interfaces/Duration move_time           # 現在までにかかった時間
    ```

> [!NOTE]
> 既存のポーズは[pose_list.yaml](sobit_light_library/config/pose_list.yaml)に確認できます．ポーズの作成方法については[ポーズの設定方法](#ポーズの設定方法)をご参照ください．

2.  `move_joint` : 指定されたジョイント(複数でも可)を任意の角度に動かします．
    ```yaml
    # MoveJoint.action
    # Goal
    string[] target_joint_names                     # 稼働関節名リスト
    float64[] target_joint_rad                      # 稼働関節角度リスト
    builtin_interfaces/Duration time_allowance      # 制限時間
    ---
    # Result
    bool success                                    # 成功/失敗
    string message                                  # 結果メッセージ
    builtin_interfaces/Duration total_elapsed_time  # かかった時間
    ---
    # Feedback
    string[] current_joint_names                    # 現在の稼働関節名リスト
    float64[] current_joint_rad                     # 現在の稼働関節角度リスト
    # float32[] current_joint_vel                   # 現在の稼働関節の速度リスト
    builtin_interfaces/Duration move_time           # 現在までにかかった時間
    ```

> [!NOTE]
> ジョイント名については[ジョイント名](#ジョイント名)をご確認ください．

3.  `move_hand_to_coord` : ハンドをxyz座標に動かします（把持モード）．
    ```yaml
    # MoveHandToTargetCoord.action
    # Goal
    geometry_msgs/TransformStamped target_coord  # 目標座標
    builtin_interfaces/Duration time_allowance   # 制限時間
    ---
    # Result
    bool success                            # 成功/失敗
    string message                          # 結果メッセージ
    geometry_msgs/Point moved_linear        # 把持に関して動いた水平距離
    float32 moved_yaw                       # 把持に関して動いた回転量
    ---
    # Feedback
    string current_state                    # 現在の動作状態
    float32 distance_to_target              # 対象物までの距離
    ```

4.  `move_hand_to_tf` : ハンドをtf名に動かします（把持モード）．
    ```yaml
    # MoveHandToTargetTF.action
    # Goal
    string target_frame                             # 把持対象のTFフレーム名
    geometry_msgs/TransformStamped tf_differential  # target_frameからの差分
    builtin_interfaces/Duration time_allowance      # 制限時間
    ---
    # Result
    bool success                               # 成功/失敗
    string message                             # 結果メッセージ
    geometry_msgs/Point moved_linear           # 把持に関して動いた水平距離
    float32 moved_yaw                          # 把持に関して動いた回転量
    ---
    # Feedback
    string current_state                       # 現在の動作状態
    float32 distance_to_target                 # 対象物までの距離
    bool object_detected                       # 対象物が検出されているか
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### ジョイント名

SOBIT LIGHTのジョイント名とその定数名を以下の通りです．

| ジョイント番号 | ジョイント名 | ジョイント定数名 |
| :---: | --- | --- |
| 0 | arm_shoulder_roll_joint  | kArmShoulderRollJoint  |
| 1 | arm_shoulder_pitch_joint | kArmShoulderPitchJoint |
| 2 | arm_elbow_pitch_joint    | kArmElbowPitchJoint    |
| 3 | arm_forearm_roll_joint   | kArmForearmRollJoint   |
| 4 | arm_wrist_pitch_joint    | kArmWristPitchJoint    |
| 5 | arm_wrist_roll_joint     | kArmWristRollJoint     |
| 6 | hand_joint               | kHandJoint             |
| 7 | head_yaw_joint           | kHeadYawJoint          |
| 8 | head_pitch_joint         | kHeadPitchJoint        |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### ポーズの設定方法

[pose_list.yaml](sobit_light_library/config/pose_list.yaml)というファイルでポーズの追加・編集ができます．以下のようなフォーマットになります．

```yaml
poses:
    - initial_pose
    - detecting_pose
    - following_pose

initial_pose:
    arm_shoulder_roll  : 0.0
    arm_shoulder_pitch : -1.5708
    arm_elbow_pitch    : 0.0
    arm_forearm_roll   : 0.0
    arm_wrist_pitch    : 0.0
    arm_wrist_roll     : 0.0
    hand               : 0.0
    head_yaw           : 0.0
    head_pitch         : 0.0
...
```  

定義したいポース名を`poses`に追加し，その後ポース名の下に各ジョイントの角度を設定します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### ホイールコントローラ

SOBIT LIGHTの移動機構(Kachaka)を動かすための情報まとめです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


#### アクション

1.  `move_wheel_linear` : 並進（前進・後退のみ）に移動させます．(弧度法：meters)
    ```yaml
    # MoveWheelLinear.action
    # Goal
    geometry_msgs/Point target_point                # 水平移動したい距離（差動二輪機構：x,全方向移動機構：x,y有効）
    builtin_interfaces/Duration time_allowance      # 制限時間
    ---
    # Result
    bool success                                    # 成功/失敗
    string message                                  # 結果メッセージ
    builtin_interfaces/Duration total_elapsed_time  # かかった時間
    ---
    # Feedback
    geometry_msgs/Point current_point               # 現在までに移動した距離
    builtin_interfaces/Duration move_time           # 現在までにかかった時間
    ```  

2.  `move_wheel_rotate` : 回転運動を行う．(弧度法：Radian)
    ```yaml
    # MoveWheelRotate.action
    # Goal
    float32 target_yaw                              # 回転したい角度
    builtin_interfaces/Duration time_allowance      # 制限時間
    ---
    # Result
    bool success                                    # 成功/失敗
    string message                                  # 結果メッセージ
    builtin_interfaces/Duration total_elapsed_time  # かかった時間
    ---
    # Feedback
    geometry_msgs/Point current_point               # 現在までに移動した距離
    builtin_interfaces/Duration move_time           # 現在までにかかった時間
    ```

</details>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## ハードウェア
SOBIT LIGHTはオープンソースハードウェアとして[OnShape](https://cad.onshape.com/documents/1c0eb7c7c35643f91262c58d/w/47103fedd1427abad418bed6/e/d36ec26c38875fb78c5b29ac)にて公開しております．

![SOBIT LIGHT in OnShape](sobit_light/docs/img/sobit_light_onshape.png)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<details>
<summary>ハードウェアの詳細についてはこちらを確認してください．</summary>

### パーツのダウンロード方法

1. Onshapeにアクセスしましょう．

> [!NOTE]
> ファイルをダウンロードするために，`OnShape`のアカウントを作成する必要はありません．ただし，本ドキュメント全体をコピーする場合，アカウントの作成を推薦します．

2. `Instances`の中にパーツを右クリックで選択します．
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

TBD

| 項目 | 詳細 |
| --- | --- |

<!-- | 最大直進速度 | 0.7[m/s] |
| 最大回転速度 | 0.229[rad/s] |
| 最大ペイロード | 0.35[kg] |
| サイズ (長さx幅x高さ) | 450x450x1250[mm] |
| 重量 | 16[kg] |
| リモートコントローラ | PS3/PS4 |
| LiDAR | UST-20LX |
| RGB-D | Azure Kinect DK (頭部)，RealSense D405 (アーム) |
| IMU | LSM6DSMUS |
| スピーカー | モノラルスピーカー |
| マイク | コンデンサーマイク |
| アクチュエータ (アーム) | 2 x XM540-W150, 6 x XM430-W320 |
| アクチュエータ (移動機構) | 4 x XM430-W320, 4 x XM430-W210 |
| 電源 | 2 x Makita 6.0Ah 18V |
| PC接続 | USB | -->

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 部品リスト（BOM）

TBD

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

- [x] OSS
    - [x] ドキュメンテーションの充実
    - [x] コーディングスタイルの統一
- [x] アクションへの対応

現時点のバッグや新規機能の依頼を確認するために[Issueページ][issues-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- CONTRIBUTING -->
<!-- ## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->


<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more NOTErmation.

<p align="right">(<a href="#readme-top">上に戻る</a>)</p> -->


<!-- 参考文献 -->
## 参考文献

* [Kachaka API](https://github.com/pf-robotics/kachaka-api)
* [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
* [ROS Humble](https://docs.ros.org/en/humble/index.html)
* [ROS2 Control](https://control.ros.org/humble/index.html)
* [ROS2 Control Gazebo](https://github.com/ros-controls/gz_ros2_control)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobit_light.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobit_light/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobit_light.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobit_light/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobit_light.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobit_light/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobit_light.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobit_light/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobit_light.svg?style=for-the-badge
[license-url]: LICENSE
