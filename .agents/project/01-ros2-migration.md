# IC-GVINS ROS2 迁移与工程现代化方案

## 状态

已确认并完成实施。目标环境为 Ubuntu 24.04、ROS2 Jazzy 和 C++20。

## 目标架构

- 使用 `ament_cmake` 和 `colcon` 构建，ROS 节点基于 `rclcpp`，启动入口为 Python launch 文件。
- ROS 包在 `ic_gvins/` 内自包含：
  - `include/ic_gvins/`：公开头文件；
  - `src/`：算法与 ROS2 实现；
  - `config/`、`launch/`：随包安装的配置与启动文件。
- 仓库级 `scripts/` 保存数据转换和评测工具；`docs/` 只保存用户文档和待确认方案；运行数据与输出不纳入源码。

## 已确认的技术决策

1. 删除 vendored Abseil，字符串、时间和格式化功能改用 C++ 标准库。
2. 删除未使用的 Boost filesystem 查找与链接，文件操作统一使用 `std::filesystem`。
3. 工程升级至 C++20。open_nav nav_core 的必要时间、地球坐标和旋转逻辑按职责并入 common，不保留独立第三方模块；迁移来源提交为 `882e8ba12889d04cbee0cf4a267518360d9df28d`。
4. `Angle`、`Earth`、`GpsTime`、`Rotation` 保持原有接口，内部直接维护项目实际使用的最小实现，包括优化器所需的四元数左/右乘矩阵。
5. `TimeCost` 继续使用 `std::chrono::steady_clock`。它是性能计时器，不使用表示绝对导航时间的 `NavTime`。
6. ROS 时间戳以 `sec`/`nanosec` 直接转换，避免先合并为大数 `double` 引入精度损失。
7. 位姿使用 `LocalBodyPose`、`GlobalBodyPose`、`CameraPose` 和 `CameraExtrinsics` 四种强类型，禁止用一个无语义的 `Pose {R,t}` 同时承载多种坐标关系。
8. VPN/虚拟网卡环境下使用 Cyclone DDS localhost 配置；配置文件随 ROS 包安装。
9. KAIST 数据转换必须使用正确的 IMU 列，并执行 FLU→FRD 转换；相机保持 OpenCV RDF 约定。
10. Urban39 配置关闭人为 GNSS 中断，固定已标定的相机/IMU 外参和硬件时间偏差。

## 已完成里程碑

- [x] ROS1 构建、节点、launch 和可视化消息迁移到 ROS2 Jazzy。
- [x] 修复 IMU 列映射、坐标轴、QoS、GNSS 状态保留和预积分边界问题。
- [x] 移除 Abseil 与 Boost 依赖，升级 C++20 并迁入所需 nav_core 逻辑。
- [x] 审计 W/N/E/B/C/V 坐标链并引入强类型位姿。
- [x] 按标准 ROS2 包目录整理源码、配置和启动文件。
- [x] 完成 common/nav_core 合并前回归测试、完整 Urban39 回放和 evo 评测。
- [x] 完成仓库内 `dataset/urban39_small` 的两次端到端复测。

## 当前整理任务

- [ ] 将已使用的 nav_core 时间、地球坐标和旋转实现分别并入现有 common 模块。
- [ ] 改用系统 glog CMake config，删除自定义 `cmake/`。
- [ ] 删除独立 `third_party/`、`test/` 目录及其构建、安装和文档引用。
- [ ] 从干净 CMake 配置完成构建与运行级检查。

## 验收边界

- 功能验收要求构建、单测、bag 完整播放、轨迹输出和干净退出全部通过。
- 精度验收使用 KAIST 参考轨迹计算平移 APE、约 1 秒平移 RPE 和旋转 RPE。
- 当前视觉/优化管线存在运行间非确定性；版本精度比较必须固定调度或重复运行并报告统计量，不能只比较一次 APE。

后续若要改变本方案，应先在 `docs/` 创建编号的待确认方案；获得确认后再更新本文件。
