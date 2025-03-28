<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

> [!WARNING]
> This robot and this repository have been supported for a short period of time and may be improved frequently and significantly in the future.

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBIT LIGHT

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#introduction">Introduction</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li>
    　<a href="#launch-and-usage">Launch and Usage</a>
      <ul>
        <li><a href="#visualization-on-rviz<">Visualization on Rviz</a></li>
        <li><a href="#visualization-on-rviz<">Run on GZ Sim</a></li>
      </ul>
    </li>
    <li>
    　<a href="#software">Software</a>
      <ul>
        <li><a href="#joint-controller">Joint Controller</a></li>
        <li><a href="#wheel-controller">Wheel Controller</a></li>
      </ul>
    </li>
    <li>
    　<a href="#hardware">Hardware</a>
      <ul>
        <li><a href="#how-to-download-3d-parts">How to download 3D Parts</a></li>
        <li><a href="#electronic-circuit-diagram">Electronic circuit Diagram</a></li>
        <li><a href="#robot-assembly">Robot Assembly</a></li>
        <li><a href="#features">Features</a></li>
        <li><a href="#bill-of-material-BOM">Bill of Material (BOM)</a></li>
      </ul>
    </li>
    <li><a href="#milestone">Milestone</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#references">References</a></li>
  </ol>
</details>



<!-- INTRODUCTION -->
## Introduction

![SOBIT LIGHT](sobit_light/docs/img/sobit_light.png)

This is a library to operate the [Kachaka](https://kachaka.life/home/)-integrated mobile manipulator (SOBIT LIGHT) developed by SOBITS.

> [!WARNING]
> If you have no previous experience controlling this robot, please have a senior colleague accompany you while you want to use this robot.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

This section describes how to set up this repository.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Prerequisites

First, please set up the following environment before proceeding to the next installation stage.

| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill |
| Python | 3.10 |
| Docker | latest |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Installation

**Development environment (local or Docker) where you will use SOBIT LIGHT:**
1. Go to the `src` folder of ROS.
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. Clone this repository.
    ```sh
    $ git clone https://github.com/TeamSOBITS/sobit_light
    ```

3. Navigate into the repository.
    ```sh
    $ cd sobit_light/
    ```

4. Install the dependent packages.
    ```sh
    $ bash install.sh
    ```

5. Compile the package.
    ```sh
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install
    $ source ~/colcon_ws/install/setup.sh
    ```

**Local Enviroment-only:**
1. Clone the Kachaka API
    ```sh
    $ cd
    $ git clone https://github.com/TeamSOBITS/kachaka-api.git
    ```

2. Build the latest Docker Image.
    ```sh
    $ cd kachaka-api/
    $ docker buildx build -t kachaka-api --target kachaka-grpc-ros2-bridge -f Dockerfile.ros2 . --build-arg BASE_ARCH=x86_64 --load
    ```

3. Let's configure `ROS_DOMAIN_IP`. In this case, we will set it to `10` as an example．
    ```sh
    $ echo 'export ROS_DOMAIN_IP=10"' >> ~/.bashrc
    $ source ~/.bashrc
    ```

> [!IMPORTANT]
> `ROS_DOMAIN_IP` must match with Local Environment and Development Environment to allow data communication within the Kachaka and the computer.

4. Check the Kachaka IP address.
    1. Ask the Kachaka directly, “ねぇカチャカ，IPアドレスを教えて (nee kachaka, IP address wo oshiete)”.\\
    Then, IP address is read out from Kachaka,
    2. or check it out from the Kachaka App.\\
    Open the `Settings` tab in the Kachaka app, tap on `App Information` in the `Settings & Information` category, and check the `IP Address` field in the `Kachaka` category.

5. Set up an alias to facilitate the connection with Kachaka and ROS Bridge.
    ```sh
    $ echo 'alias kachaka="bash ~/kachaka-api/tools/ros2_bridge/start_bridge.sh"' >> ~/.bashrc
    $ source ~/.bashrc
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- LAUNCH AND USAGE EXAMPLES -->
## Launch and Usage

1. Bring up the ROS Bridge to connect Kachaka in your **local environment**.
    ```
    $ kachaka <Kachaka IP> sobit_light no
    ```

> [!NOTE]
> By writing `sobit_light`, you are setting the `namespace` of the robot. Additionally, `no` stops Kachaka from publishing the robot_description. For more details, please refer to [Starting ros2_bridge using Docker](https://github.com/TeamSOBITS/kachaka-api/blob/main/docs/ROS2.md#%E3%83%96%E3%83%AA%E3%83%83%E3%82%B8%E3%81%AE%E8%B5%B7%E5%8B%95).

> [!WARNING]
> Please note that the Kachaka IP might have changed.

2. Execute the launch file [minimal.launch](sobit_light_bringup/launch/minimal.launch.py) in your **development environment**.
    ```sh
   $ ros2 launch sobit_light_bringup real_minimal.launch.py
    ```

3. If you did not succeed in connecting to Kachaka, check the following points:

    - Ensure the emergency stop button is not pressed.
    - Verify the battery is sufficiently charged.
    - Confirm the USB hub is connected to the computer.
    - [TODO] Check if the Dynamixel Dongle is named `/dev/ttyUSB0`.
    - - To verify, run `$ ls /dev` and if `/dev/ttyUSB1` is displayed, update the `usb_port` in [controllers.urdf.xacro](sobit_light_description/urdf/controllers.urdf.xacro).
    - Ensure the Kachaka IP is correct.
    - Verify that the `ROS_DOMAIN_ID` is the same on both the Kachaka and the development environment.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Visualize on Rviz2

As a preliminary step to running the actual machine, SOBIT LIGHT can be visualized on Rviz to display the robot's configuration.

```sh
$ ros2 launch sobit_light_description display.launch.py
```

If it works correctly, Rviz will be displayed as follows.
![SOBIT LIGHT Display with Rviz](sobit_light/docs/img/sobit_light_rviz.png)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Run on GZ Sim

SOBIT LIGHT has a simulation environment with Gazebo Fortress, allowing you to verify operations even without the actual machine.

```sh
$ ros2 launch sobit_light_bringup gz_minimal.launch.py
```

If it works correctly, the following Gazebo screen will be displayed.
![SOBIT LIGHT Gazebo Fortress](sobit_light/docs/img/sobit_light_gz_sim.png)

> [!WARNING]
> Since it is equipped with sensors similar to the actual machine, the processing may become heavy depending on the computer. Please select only the necessary sensors in [gz_minimal.launch.py](sobit_light_bringup/launch/gz_minimal.launch.py).

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

Additionally, multiple SOBIT LIGHTs can be spawned in the same simulation environment. To do this, configure [gz_minimal.launch.py](sobit_light_bringup/launch/gz_minimal.launch.py) to execute `gz_robot.launch.py` according to the number of robots.

Please, make sure that `robot_name` must have a different value among robots.
Moreover, you can change the spawining coordinates of the robot in `robot_coords_x`, `robot_coords_y` and `robot_coords_z`.

Here is an example.

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

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Software

<details>
<summary>Summary of information on SOBIT LIGHT and related software</summary>


### Joint Controller

This is a summary of information for moving the pan-tilt mechanism and manipulators of SOBIT LIGHT.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### Movement Methods

1.  `move_to_pose` : Move it to a predetermined pose.
    ```yaml
    # MoveToPose.action
    # Goal
    string pose_name                                # Target pose name
    builtin_interfaces/Duration time_allowance      # Target time length
    ---
    # Result
    bool success                                    # Success / Failure
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # Finished time length
    ---
    # Feedback
    string[] current_joint_names                    # Currently moving joint name(s)
    float32[] current_joint_rad                     # Currently moving joint position(s)
    # float32[] current_joint_vel                   # Currently moving joint velocity(s)
    builtin_interfaces/Duration move_time           # Elapsed time length
    ```

> [!NOTE]
> Existing poses can be found in [pose_list.yaml](sobit_light_library/config/pose_list.yaml). Please refer to [How to set new poses](#how-to-set-new-poses) for how to create poses.

2.  `move_joint` : Moves any joint to an arbitrary angle.
    ```yaml
    # MoveJoint.action
    # Goal
    string[] target_joint_names                     # Target joint name(s)
    float64[] target_joint_rad                      # Target joint position(s)
    builtin_interfaces/Duration time_allowance      # Target time length
    ---
    # Result
    bool success                                    # Success / Failure
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # Finished time length
    ---
    # Feedback
    string[] current_joint_names                    # Currently moving joint name(s)
    float64[] current_joint_rad                     # Currently moving joint position(s)
    # float32[] current_joint_vel                   # Currently moving joint velocity(s)
    builtin_interfaces/Duration move_time           # Elapsed time length
    ```

> [!NOTE]
> Please check the previously defined joint names in the [Joints Name](#joints-name) section.
 
3.  `move_hand_to_coord` : Move the hand to xyz coordinates (grasp mode).
    ```yaml
    # MoveHandToTargetCoord.action
    # Goal
    geometry_msgs/TransformStamped target_coord  # Target coordinates
    builtin_interfaces/Duration time_allowance   # Target time length
    ---
    # Result
    bool success                            # Success / Failure
    string message                          # Result message
    geometry_msgs/Point moved_linear        # Move linear to grasp
    float32 moved_yaw                       # Move yaw to grasp
    ---
    # Feedback
    string current_state                    # Current state message
    float32 distance_to_target              # Distance to the target object
    ```

4.  `move_hand_to_tf` : Moves the hand to the tf name (grasp mode).
    ```yaml
    # MoveHandToTargetTF.action
    # Goal
    string target_frame                             # Target TF name
    geometry_msgs/TransformStamped tf_differential  # Target TF-related shift
    builtin_interfaces/Duration time_allowance      # Target time length
    ---
    # Result
    bool success                               # Success / Failure
    string message                             # Result message
    geometry_msgs/Point moved_linear           # Move linear to grasp
    float32 moved_yaw                          # Move yaw to grasp
    ---
    # Feedback
    string current_state                       # Current state message
    float32 distance_to_target                 # Distance to the target object
    bool object_detected                       # Object Detection Flag
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### Joints name

The joint names of SOBIT LIGHT and their constants are listed below.

| Joint Number | Joint Name | Joint Constant Name |
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

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### How to set new poses

Poses can be added and edited in the file [pose_list.yaml](sobit_light_library/config/pose_list.yaml). The format is as follows:

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

Add the desired pose name to `poses`, and then set the angles for each joint under the pose name.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Wheel Controller

This is a summary of information for moving the SOBIT LIGHT moving mechanism.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


#### Moving Methods

1.  `move_wheel_linear` : Perform translational motion (straight-line only).
    ```yaml
    # MoveWheelLinear.action
    # Goal
    geometry_msgs/Point target_point                # Target Translational Distance
    builtin_interfaces/Duration time_allowance      # Target time length
    ---
    # Result
    bool success                                    # Success / Failure
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # Finished time length
    ---
    # Feedback
    geometry_msgs/Point current_point               # Currently displaced distance
    builtin_interfaces/Duration move_time           # Currently elapsed time
    ```  

2.  `move_wheel_rotate` : Perform rotational motion (units: Radian)
    ```yaml
    # MoveWheelRotate.action
    # Goal
    float32 target_yaw                              # Target Rotational Distance
    builtin_interfaces/Duration time_allowance      # Target time length
    ---
    # Result
    bool success                                    # Success / Failure
    string message                                  # Result message
    builtin_interfaces/Duration total_elapsed_time  # Finished time length
    ---
    # Feedback
    geometry_msgs/Point current_point               # Currently displaced distance
    builtin_interfaces/Duration move_time           # Currently elapsed time
    ```

</details>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Hardware

SOBIT LIGHT is available as open source hardware at [OnShape](https://cad.onshape.com/documents/1c0eb7c7c35643f91262c58d/w/47103fedd1427abad418bed6/e/d36ec26c38875fb78c5b29ac).

![SOBIT LIGHT in OnShape](sobit_light/docs/img/sobit_light_onshape.png)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<details>
<summary>For more information on hardware, please click here.</summary>

### How to download 3D parts

1. Access Onshape.

> [!NOTE]
> You do not need to create an `OnShape` account to download files. However, if you wish to copy the entire document, we recommend that you create an account.

2. Select the part in `Instances` by right-clicking on it.
3. A list will be displayed, press the `Export` button.
4. In the window that appears, there is a `Format` item. Select `STEP`.
5. Finally, press the blue `Export` button to start the download.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Electronic Circuit Diagram

TBD

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Robot Assembly

TBD

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Features

TBD

| Item | Details |
| --- | --- |
<!-- | Maximum linear velocity | 0.7[m/s] |
| Maximum Rotational Speed | 0.229[rad/s] |
| Maximum Payload | 0.35[kg] |
| Size (LxWxH) | 450x450x1250[mm] |
| Weight | 16[kg] |
| Remote Controller | PS3/PS4 |
| LiDAR | UST-20LX |
| RGB-D | Azure Kinect DK (head), RealSense D405 (arm) |
| IMU | LSM6DSMUS |
| Speaker | Mono Speaker |
| Microphone | Condenser Microphone |
| Actuator (Arm) | 2 x XM540-W150, 6 x XM430-W320 |
| Actuator (movement mechanism) | 4 x XM430-W320, 4 x XM430-W210 |
| Power Supply | 2 x Makita 6.0Ah 18V |
| PC Connection | USB | -->

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Bill of Materials (BOM)

TBD

| Part | Model Number | Quantity | Where to Buy |
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

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MILESTONE -->
## Milestone

- [x] OSS
    - [x] Improved documentation
    - [x] Unified coding style
- [x] Add support to Action Communication

See the [open issues][issues-url] for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>


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

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->


<!-- LICENSE -->
<!-- ## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p> -->


<!-- ACKNOWLEDGMENTS -->
## References

* [Kachaka API](https://github.com/pf-robotics/kachaka-api)
* [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
* [ROS Humble](https://docs.ros.org/en/humble/index.html)
* [ROS2 Control](https://control.ros.org/humble/index.html)
* [ROS2 Control Gazebo](https://github.com/ros-controls/gz_ros2_control)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



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
