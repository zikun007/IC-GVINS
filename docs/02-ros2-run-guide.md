# IC-GVINS ROS2 Jazzy 运行与 urban39 评测指南

本文档的 apt 安装与 `/opt/ros` 示例适用于 Ubuntu 24.04。Apple Silicon macOS 请先使用 [README 的 Pixi 工作流](../README.md#16-native-macos-apple-silicon-with-pixi)；不要在 macOS 执行下文的 apt 命令。以下数据路径来自 Linux 验证环境：

- 项目：`/home/kun24/IC-GVINS`
- KAIST urban39：`/mnt/OpenDataset/kaist-urban39`
- 已生成并保留的完整 ROS2 bag：`/mnt/OpenDataset/kaist-urban39/urban39_ros2_bag_fixed`

完整 bag 时长约 1866.87 秒、大小 37.5 GiB，包含 `/imu0`、`/gnss0` 和 `/cam0`。正常评测应按 1 倍速回放，至少需要约 31 分钟。

## 1. 安装依赖

安装 ROS2、编译库、Cyclone DDS 和 RViz2 依赖：

```zsh
sudo apt update
sudo apt install \
  python3-colcon-common-extensions python3-numpy python3-pil \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rviz2 \
  libeigen3-dev libopencv-dev libceres-dev libgoogle-glog-dev \
  libyaml-cpp-dev libtbb-dev
```

使用 `uv` 安装 evo，不要再创建单独的 `venv`：

```zsh
uv tool install evo
uv tool ensurepath
```

首次执行 `uv tool ensurepath` 后若当前终端还找不到 `evo_ape`，重新打开终端即可。

## 2. 编译项目

当前版本使用 C++20。导航时间、WGS84/ECEF/NED、固定世界系和旋转实现已经并入
`ic_gvins/src/common/` 与 `ic_gvins/include/ic_gvins/common/`，编译和运行不依赖外部 open_nav 源码目录。

```zsh
export IC_GVINS_ROOT=/home/kun24/IC-GVINS
cd "$IC_GVINS_ROOT"
source /opt/ros/jazzy/setup.zsh

colcon build \
  --symlink-install \
  --packages-select ic_gvins \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source "$IC_GVINS_ROOT/install/setup.zsh"
```

后续每个新终端都要重新加载 ROS2 和工作空间环境。

## 3. 配置 Cyclone DDS

VPN 和虚拟网卡可能干扰 DDS 自动选网卡。urban39 在同一台机器上回放时，两个终端都使用仓库提供的仅本机配置：

```zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_AUTOMATIC_DISCOVERY_RANGE
export IC_GVINS_CONFIG="$(ros2 pkg prefix --share ic_gvins)/config"
export CYCLONEDDS_URI="file://$IC_GVINS_CONFIG/cyclonedds_localhost.xml"
```

该配置只允许回环接口通信。如果需要多机 ROS2 通信，执行 `unset CYCLONEDDS_URI`，并根据实际局域网重新配置发现范围和网卡。

## 4. 启动 IC-GVINS

打开终端 1：

```zsh
export IC_GVINS_ROOT=/home/kun24/IC-GVINS
cd "$IC_GVINS_ROOT"
source /opt/ros/jazzy/setup.zsh
source "$IC_GVINS_ROOT/install/setup.zsh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_AUTOMATIC_DISCOVERY_RANGE
export IC_GVINS_CONFIG="$(ros2 pkg prefix --share ic_gvins)/config"
export CYCLONEDDS_URI="file://$IC_GVINS_CONFIG/cyclonedds_localhost.xml"

ros2 launch ic_gvins ic_gvins.launch.py \
  configfile:="$IC_GVINS_CONFIG/kaist-urban39.yaml" \
  use_rviz:=true
```

若不需要图形界面，或当前会话没有桌面显示，把最后一项改为 `use_rviz:=false`。

当前 urban39 配置已确认：GNSS 仿真中断关闭，相机内外参来自数据集标定，IMU 为 100 Hz，KAIST FLU 数据在生成 bag 时转换为 IC-GVINS 使用的 FRD 坐标系。

## 5. 回放完整 urban39

确认终端 1 中的节点已经启动，再打开终端 2：

```zsh
export IC_GVINS_ROOT=/home/kun24/IC-GVINS
export URBAN39_ROOT=/mnt/OpenDataset/kaist-urban39
cd "$IC_GVINS_ROOT"
source /opt/ros/jazzy/setup.zsh
source "$IC_GVINS_ROOT/install/setup.zsh"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_AUTOMATIC_DISCOVERY_RANGE
export IC_GVINS_CONFIG="$(ros2 pkg prefix --share ic_gvins)/config"
export CYCLONEDDS_URI="file://$IC_GVINS_CONFIG/cyclonedds_localhost.xml"

ros2 bag play "$URBAN39_ROOT/urban39_ros2_bag_fixed" \
  --rate 1.0 \
  --delay 3 \
  --disable-keyboard-controls \
  --qos-profile-overrides-path "$IC_GVINS_CONFIG/rosbag_qos_reliable.yaml"
```

bag 播放完成后，在终端 1 按 `Ctrl+C`，让节点正常关闭并刷新输出文件。结果会写入：

```text
/home/kun24/IC-GVINS/output/urban39/T年月日时分秒/
```

其中 `trajectory.csv` 是 evo 评测所需的估计轨迹。

## 6. 重新生成 ROS2 bag（仅当完整 bag 丢失时）

现有 `urban39_ros2_bag_fixed` 已经可用，不需要重复生成。若以后删除了它，可从原始 CSV 和 PNG 重新生成：

```zsh
export IC_GVINS_ROOT=/home/kun24/IC-GVINS
export URBAN39_ROOT=/mnt/OpenDataset/kaist-urban39
cd "$IC_GVINS_ROOT"
source /opt/ros/jazzy/setup.zsh

/usr/bin/python3 scripts/kaist_urban39_to_rosbag.py \
  "$URBAN39_ROOT" \
  "$URBAN39_ROOT/urban39_ros2_bag_fixed"
```

脚本按时间合并三路数据，执行 Bayer BGGR 解码，并把 IMU 从 FLU 转为 FRD。输出目录已存在时脚本会拒绝覆盖，防止误删有效 bag。

可用下面的命令检查生成结果：

```zsh
source /opt/ros/jazzy/setup.zsh
ros2 bag info /mnt/OpenDataset/kaist-urban39/urban39_ros2_bag_fixed
```

正确的完整包应有 207206 条消息：`/imu0` 186682 条、`/gnss0` 1867 条、`/cam0` 18657 条。

## 7. 使用 evo 评测

先查看运行目录并把本次目录填入变量：

```zsh
ls -dt /home/kun24/IC-GVINS/output/urban39/T*/
export IC_GVINS_RUN=/home/kun24/IC-GVINS/output/urban39/T年月日时分秒
export URBAN39_ROOT=/mnt/OpenDataset/kaist-urban39
```

转换估计轨迹与参考轨迹：

```zsh
cd /home/kun24/IC-GVINS
/usr/bin/python3 scripts/prepare_kaist_evo.py \
  "$IC_GVINS_RUN/trajectory.csv" \
  "$URBAN39_ROOT/urban39-pankyo_pose/global_pose.csv" \
  "$URBAN39_ROOT/urban39-pankyo_calibration/calibration/Vehicle2IMU.txt" \
  "$IC_GVINS_RUN/evo"
```

计算平移 APE、约 1 秒的平移 RPE 和旋转 RPE。`trajectory.csv` 约为 10 Hz，因此这里使用 10 帧间隔：

```zsh
evo_ape tum \
  "$IC_GVINS_RUN/evo/reference_imu_ned.tum" \
  "$IC_GVINS_RUN/evo/estimate_imu_ned.tum" \
  -a -r trans_part --t_max_diff 0.02

evo_rpe tum \
  "$IC_GVINS_RUN/evo/reference_imu_ned.tum" \
  "$IC_GVINS_RUN/evo/estimate_imu_ned.tum" \
  -a -r trans_part -d 10 -u f --t_max_diff 0.02

evo_rpe tum \
  "$IC_GVINS_RUN/evo/reference_imu_ned.tum" \
  "$IC_GVINS_RUN/evo/estimate_imu_ned.tum" \
  -a -r angle_deg -d 10 -u f --t_max_diff 0.02
```

如需轨迹图，在任一命令末尾加 `-p --plot_mode xy`。

## 8. 常见问题

- 终端之间看不到 topic：确认两个终端的 `RMW_IMPLEMENTATION` 和 `CYCLONEDDS_URI` 完全一致，并先执行 `ros2 daemon stop` 再重试。
- VPN 开启后无法通信：保留上面的 loopback Cyclone DDS 配置，不要让 DDS 自动选择 VPN 网卡。
- RViz2 无法启动：先确认已安装 `ros-jazzy-rviz2`，无桌面环境时用 `use_rviz:=false`。
- 节点没有输出：确认从项目根目录启动，因为 YAML 中的 `outputpath` 是相对路径 `./output/urban39`。
- 运行过快导致图像或 GNSS 排队：使用 `--rate 1.0` 和文档中的 QoS 覆盖文件，不建议用更高倍速做精度评测。

若只想清理后重新编译，可在项目根目录删除 `build/`、`install/`、`log/` 和 `output/`；这些目录均为可再生文件，不包含源码。

## 9. 时间与坐标实现维护说明

导航时间和坐标实现已直接并入 common：`GpsTime` 负责 GPS/Unix 时间和闰秒转换，`Earth` 负责 WGS84/ECEF/NED 与固定世界系，`Rotation` 负责通用姿态转换。算法代码继续使用原有接口。

这些实现源自 open_nav/nav_core 迁移版本 `882e8ba12889d04cbee0cf4a267518360d9df28d`。后续若同步上游，应按 common 的职责逐项审查，重新构建并完整回放 Urban39，不要整文件覆盖。`TimeCost` 是性能计时器，继续使用 `std::chrono::steady_clock`。
