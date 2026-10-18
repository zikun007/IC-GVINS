# 关键调试与验证记录

## 已解决问题

| 问题 | 原因 | 处理结果 |
|---|---|---|
| VPN 下 ROS2 进程发现异常 | DDS 在虚拟网卡环境中的接口选择不稳定 | 使用 `rmw_cyclonedds_cpp` 和 localhost Cyclone DDS 配置 |
| KAIST IMU 解算异常 | CSV 列映射错误且数据为 FLU，估计器要求 FRD | 修正 gyro/accel 列并在转换脚本中执行 FLU→FRD |
| 低视差/停车段 GNSS 丢失 | GNSS 插入与视觉关键帧类型耦合，同时间视觉节点覆盖 GNSS 状态 | GNSS 独立插入；同时间节点保留 GNSS 数据与约束 |
| 预积分和 Ceres 崩溃 | 倒序/越界 IMU 区间、Eigen 临时对象生命周期、空或重复边缘化参数块 | 增加边界与唯一性检查，并显式求值 Eigen 表达式 |
| ROS bag 通信不稳定 | 传感器 QoS 与播放器默认策略不匹配、缓存不足 | 提供 Reliable QoS 覆盖并扩大可靠队列 |
| Abseil/Boost 冗余依赖 | 仅使用通用工具功能，Boost filesystem 实际未被源码使用 | 以标准库替换并从 CMake、文档和产物中移除依赖 |
| Pose 坐标语义容易混淆 | 单一 `Pose {R,t}` 同时表示 body、camera、global 等关系 | 拆分四种强类型，并为坐标链增加回归测试 |
| macOS 编译找不到 `ulong` / `usleep` | 依赖 Linux 类型别名与间接 POSIX 头文件 | ID 统一为 `std::uint64_t`，等待使用 `std::this_thread::sleep_for` |
| Debug 构建仍被优化 | 两个目标硬编码 `-O3` | 移除目标级优化选项，由 `CMAKE_BUILD_TYPE` 控制 |
| Pixi 构建混入 Homebrew 的 TBB/yaml-cpp | TBB 缺少开发包，yaml-cpp 使用裸库名 | 添加 `tbb-devel`；显式 Pixi 前缀、排除 Homebrew；使用 `yaml-cpp::yaml-cpp` |
| macOS 节点启动时报 `_PyExc_RuntimeError` | ROS/rclcpp 导出依赖包含无用的 `rosidl_generator_py`，要求 Python 宿主 | 节点显式链接 C++ typesupport，并在 Apple 链接时使用 `-dead_strip_dylibs` 移除无用动态库；不预加载 Python |
| 默认 DDS 报线程亲和性错误，loopback 配置无法发现节点 | Fast DDS 的 macOS 线程配置；禁用组播后缺少显式本机 peer | Pixi 选择 Cyclone DDS；接口按 `127.0.0.1` 匹配，添加本机单播 peer；ROS 图查询通过 |
| OpenCV 不加组件时链接全部扩展模块 | `find_package(OpenCV REQUIRED)` 默认导出完整模块集合 | 仅声明 core、imgproc、calib3d、video，避免无用依赖 |

## 坐标与时间结论

- W 为首个 GNSS 原点固定 NED，N 为当前位置 NED，B 为 IMU FRD，C 为相机 RDF，V 为 KAIST FLU。
- 状态位姿为 `T_w_b`，相机外参为 `T_b_c`，相机位姿为 `T_w_c = T_w_b * T_b_c`。
- `global2local` 使用 `R_n0_e (p_e - p_e0)`；未发现 NED/ENU 或 `T_b_c/T_c_b` 方向反转。
- ROS2 时间戳直接由 sec/nsec 进入 `NavTime`；Urban39 所在年代的 GPS/UTC 转换与旧实现一致。
- nav_core 的 `quaternionleft/right` 缺失，继续由 IC-GVINS 兼容层维护。

## 精度记录

| 数据与版本 | 位姿数 | APE RMSE | 约 1 秒平移 RPE | 约 1 秒旋转 RPE |
|---|---:|---:|---:|---:|
| ROS2 迁移早期短片段 | 1726 | 1.862069 m | 0.451628 m | 0.238383° |
| nav_core 后短跑 | 1711 | 2.064512 m | 0.466410 m | 0.240954° |
| `urban39_small` 第一次 | 1726 | 2.304811 m | 0.523319 m | 0.246255° |
| `urban39_small` 第二次 | 1726 | 2.052522 m | 0.461032 m | 0.232099° |
| nav_core 后完整 31 分钟 | 18595 | 12.794571 m | 3.114232 m | 0.174006° |
| 旧完整运行重算 | 18595 | 10.884401 m | 1.748555 m | 0.162246° |

短跑之间已经出现 0.7–1.4 m 量级的直接位置 RMSE，因此上述单次指标不能独立证明版本精度变好或变差。

## 已知命令注意事项

- ROS2 Jazzy 不支持 `--duration`；本次 Pixi 版本已实际使用 `--playback-duration 30` 完成有界回放。
- `colcon` 的 `--log-base` 必须放在主命令之前。
- Cyclone DDS 配置路径应指向包内 `ic_gvins/config/cyclonedds_localhost.xml` 或安装后的 share 路径。
- 带 daemon 的 ROS2 图检查曾因 daemon 状态超时；排查发现问题时优先使用 `--no-daemon` 做隔离验证。

## macOS 验证（2026-10-18）

- `pixi install --locked`、Debug 编译、ROS launch、三路订阅与 loopback DDS 发现均已通过。
- LLDB 实际命中 `fusion_ros.cc:250` 的 `main`，显示 `argc=4` 与源码栈帧。
- Urban39 前 30 秒、0.5 倍速回放：226 个有效位姿，时间戳严格递增；GNSS/INS、视觉跟踪与优化路径均已执行，两个 launch 子节点均正常退出。
- 冒烟脚本只向 Pixi 父进程发送一次 SIGINT，由其转发；同时向整个进程组发信号会造成重复转发，曾使静态 TF 进程在第二次信号后退出为 -2。该问题属于临时验证脚本，不是节点修复；脚本已在验证后删除。

