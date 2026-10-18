# IC-GVINS

## A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System

Visual navigation systems are susceptible to complex environments, while inertial navigation systems (INS) are not affected by external factors. Hence, we present IC-GVINS, a robust, real-time, INS-centric global navigation satellite system (GNSS)-visual-inertial navigation system to fully utilize the INS advantages. The Earth rotation has been compensated in the INS to improve the accuracy of high-grade inertial measurement units (IMUs). To promote the system robustness in high-dynamic conditions, the precise INS information is employed to assist the feature tracking and landmark triangulation. With a GNSS-aided initialization, the IMU, visual, and GNSS measurements are tightly fused in a unified world frame within the factor graph optimization framework.

<img src="resources/overview.png" alt="overview" style="zoom: 30%;" />

**Authors:** Hailiang Tang, Xiaoji Niu, and Tisheng Zhang from the [Integrated and Intelligent Navigation (i2Nav) Group](http://www.i2nav.com/), Wuhan University.

**Related Paper:**

- Xiaoji Niu, Hailiang Tang, Tisheng Zhang, Jing Fan, and Jingnan Liu, “IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System,” *IEEE Robotics and Automation Letters*, 2022.
- Hailiang Tang, Tisheng Zhang, Xiaoji Niu, Jing Fan, and Jingnan Liu, “Impact of the Earth Rotation Compensation on MEMS-IMU Preintegration of Factor Graph Optimization,” *IEEE Sensors Journal*, 2022.

**Related Video**:

Click the following image to open our video on [Bilibili](https://www.bilibili.com/video/BV15a411q71D).
<a href ="https://www.bilibili.com/video/BV15a411q71D"><img src="resources/cover.png" alt="cover" style="zoom: 30%;" /></a>

**Contacts:**

- For any technique problem, you can send an email to Dr. Hailiang Tang (thl@whu.edu.cn).
- For Chinese users, we also provide a QQ group (481173293) for discussion. You are required to provide your organization and name.

## 1 Prerequisites

### 1.1 System and compiler

This ROS2 port targets Ubuntu 24.04 and ROS 2 Jazzy with a C++17 compiler.

```shell
sudo apt install build-essential python3-colcon-common-extensions
```

### 1.2 Robot Operating System (ROS)

Install ROS 2 Jazzy, then source `/opt/ros/jazzy/setup.bash`. The optional visualizer needs `ros-jazzy-rviz2`.

### 1.3 Ceres Solver with its Dependencies

We use Ceres Solver to solve the non-linear least squares problem in IC-GVINS. The supported version is **Ceres Solver 2.0.0 or 2.1.0**. Please follow [Ceres installation instructions](http://ceres-solver.org/installation.html).

The dependencies **Eigen (>=3.3.7)**, **TBB**, **glog (>=0.4.0)** are also used in IC-GVINS. You can install them as follows:

```shell
sudo apt install libeigen3-dev libgoogle-glog-dev libtbb-dev
```

If the version cannot be satisfied in your system repository, you should build them from the source code.

### 1.4 OpenCV

The supported version is **OpenCV (>=3.2.0)**. You can install OpenCV from your system repository or build from the source code. OpenCV 4 is also supported in IC-GVINS.

```
sudo apt install libopencv-dev
```

### 1.5 yaml-cpp

```shell
sudo apt install libyaml-cpp-dev
```

## 2 Build and run IC-GVINS

For the tested ROS2 Jazzy + KAIST urban39 workflow, including Cyclone DDS,
RViz2, bag generation and evo commands, see
[`docs/02-ros2-run-guide.md`](docs/02-ros2-run-guide.md).

### 2.1 Build the source code

```shell
# Make a colcon workspace
mkdir -p ~/gvins_ws/src && cd ~/gvins_ws/src

# Clone the repository into src directory
git clone https://github.com/i2Nav-WHU/IC-GVINS.git

# To gvins_ws directory
cd ..

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ic_gvins
```

### 2.2 Run demo dataset

If you have already downloaded the open-sourced dataset, run the following commands.

```shell
# Open a terminal and source the workspace environments
source /opt/ros/jazzy/setup.bash
source ~/gvins_ws/install/setup.bash

# Run IC-GVINS node
# You should change the path in both the configuration file and command line
ros2 launch ic_gvins ic_gvins.launch.py configfile:=/absolute/path/gvins.yaml

# Open another terminal to play the ROS bag
ros2 bag play /absolute/path/dataset_bag
```

For the supplied raw KAIST urban39 directory, create a ROS2 bag and matching configuration first:

```shell
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 scripts/kaist_urban39_to_rosbag.py /mnt/OpenDataset/kaist-urban39 /tmp/urban39_bag
/usr/bin/python3 scripts/generate_kaist_urban39_config.py /mnt/OpenDataset/kaist-urban39 ic_gvins/config/kaist-urban39.yaml
ros2 launch ic_gvins ic_gvins.launch.py configfile:=$PWD/ic_gvins/config/kaist-urban39.yaml use_rviz:=true
# In a second terminal:
ros2 bag play /tmp/urban39_bag
```

`kaist_urban39_to_rosbag.py` debayers the left camera as BGGR and converts the KAIST IMU from FLU to the FRD convention required by IC-GVINS. Use `--bayer-pattern` or `--keep-flu` only when your data variant requires it.

The exact `R_a_b`/`t_a_b` convention and the W/N/E/B/C/V frame mapping are documented in [docs/03-coordinate-frames.md](docs/03-coordinate-frames.md).

The ROS2 package is self-contained under `ic_gvins/`: public headers are in
`include/ic_gvins/`, implementation files in `src/`, and launch/configuration
assets in `launch/` and `config/`. Repository-level dataset tools remain in
`scripts/`, while documentation and figures remain in `docs/` and `resources/`.

After a full replay, install evo with uv, then convert the two trajectories and evaluate translation errors:

```shell
uv tool install evo
/usr/bin/python3 scripts/prepare_kaist_evo.py /path/to/output/trajectory.csv \
  /mnt/OpenDataset/kaist-urban39/urban39-pankyo_pose/global_pose.csv \
  /mnt/OpenDataset/kaist-urban39/urban39-pankyo_calibration/calibration/Vehicle2IMU.txt /tmp/urban39_evo
evo_ape tum /tmp/urban39_evo/reference_imu_ned.tum /tmp/urban39_evo/estimate_imu_ned.tum -a --pose_relation trans_part
# IC-GVINS writes one trajectory pose per 10 IMU updates. With urban39's
# 100 Hz IMU, the output trajectory is 10 Hz, so 10 frames is about 1 s.
evo_rpe tum /tmp/urban39_evo/reference_imu_ned.tum /tmp/urban39_evo/estimate_imu_ned.tum -a --pose_relation trans_part --delta 10 --delta_unit f
```

## 3 Datasets

### 3.1 Format

We use standard ROS2 bag for IC-GVINS. The employed messages are as follows:

| Sensor   | Message                                                                                 | Default Topic | KAIST Dataset (Hz) | IC-GVINS Dataset (Hz) |
| ---------- | ----------------------------------------------------------------------------------------- | --------------- | -------------------- | ----------------------- |
| Camera   | [sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)         | /cam0         | 10                 | 20                    |
| IMU      | [sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)             | /imu0         | 100                | 200                   |
| GNSS-RTK | [sensor_msgs/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) | /gnss0        | 1                  | 1                     |

The IMU should be in **front-right-down** format in the IC-GVINS.

### 3.2 KAIST Complex Urban Dataset

The tested sequences are *urban38* and *urban39*.

| Sequence         | Time length (seconds) | Trajectory Length (m) | Baidu Cloud Link                                                      |
| ------------------ | ----------------------- | ----------------------- | ----------------------------------------------------------------------- |
| urban38 (top)    | 2154                  | 11191                 | [urban38.bag](https://pan.baidu.com/s/1CJj0Z1vClU4aL8zSna-LzQ) (gyvr) |
| urban39 (bottom) | 1856                  | 10678                 | [urban39.bag](https://pan.baidu.com/s/14CHl7LaIIkBKpwhyuPPbPA) (mnrn) |

<img src="resources/dataset/urban38.png" alt="urban38" style="zoom: 50%;" />

<img src="resources/dataset/urban39.png" alt="urban39" style="zoom: 50%;" />

### 3.3 IC-GVINS Robot Dataset

We also open source our self-collected robot dataset.

| Sequence          | Time length (seconds) | Trajectory Length (m) | Baidu Cloud Link                                                       |
| ------------------- | ----------------------- | ----------------------- | ------------------------------------------------------------------------ |
| campus (top)      | 950                   | 1337                  | [campus.bag](https://pan.baidu.com/s/18yRYUQdu_-DmrYnXQy9VNQ) (igks)   |
| building (bottom) | 1820                  | 2560                  | [building.bag](https://pan.baidu.com/s/1Y48jFmdAOBF4y30KBK9bAw) (2drg) |

<img src="resources/dataset/campus.png" alt="campus" style="zoom: 50%;" />

<img src="resources/dataset/building.png" alt="building" style="zoom: 50%;" />

### 3.4 Your own dataset

You can run IC-GVINS with your self-collected dataset. Keep in mind the following notes:

1. You should prepare well-synchronized GNSS, Camera, and IMU data in a ROS bag;
2. The IMU data should be in front-right-down format;
3. Modify the topic names in the ROS2 launch file;
4. Modify the parameters in the configuration file.

### 3.5 Evaluation

We use [evo](https://github.com/MichaelGrupp/evo) to evaluate the TUM trajectory files. We also provide some useful scripts ([evaluate_odometry](https://github.com/i2Nav-WHU/evaluate_odometry)) for evaluation.

## 4 Acknowledgements

We thanks the following projects for the helps in developing and evaluating the IC-GVINS:

- [OB_GINS](https://github.com/i2Nav-WHU/OB_GINS): An Optimization-Based GNSS/INS Integrated Navigation System
- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion): An optimization-based multi-sensor state estimator
- [Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset): Complex Urban Dataset with Multi-level Sensors from Highly Diverse Urban Environments
- [evo](https://github.com/MichaelGrupp/evo): Python package for the evaluation of odometry and SLAM

## 5 License

The source code is released under GPLv3 license.

We are still working on improving the code. For any technical issues, please contact Dr. Hailiang Tang ([thl@whu.edu.cn](mailto:thl@whu.edu.cn)) or open an issue at this repository.

For commercial usage, please contact Prof. Xiaoji Niu ([xjniu@whu.edu.cn](mailto:xjniu@whu.edu.cn)).
