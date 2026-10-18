# IC-GVINS 当前状态

最后更新：2026-10-18

当前任务：按用户确认完成 Apple Silicon macOS 原生 Pixi/ROS2 Jazzy 编译环境，并验证节点运行与 LLDB 调试。

## 本次文档维护

- 已按 `AGENTS.md` 将确认后的 ROS2 迁移方案归档到 `.agents/project/01-ros2-migration.md`。
- 已将当前交付状态、验证结果、调试结论和遗留风险提炼到 `.agents/memory/`。
- 已删除 `docs/` 中两套重复且部分过期的 planning 文档；`docs/` 现在只保存用户文档、索引和未来的待确认方案。

## 交付状态

- ROS2 Jazzy、C++20、nav_core、强类型位姿、Abseil/Boost 清理和标准 ROS2 目录整理均已完成。
- 当前构建命令、启动方式、Urban39 回放和 evo 评测见 `docs/02-ros2-run-guide.md`。
- 坐标系和位姿方向见 `docs/03-coordinate-frames.md`。
- ROS 包源码位于 `ic_gvins/`，运行配置位于 `ic_gvins/config/`。
- 完整 Urban39 bag 位于 `/mnt/OpenDataset/kaist-urban39/urban39_ros2_bag_fixed`；仓库内验证片段为 `dataset/urban39_small`。

## macOS 原生开发环境

- 新增 `pixi.toml`、`pixi.lock`，平台为 `osx-arm64`；`.pixi/` 不纳入版本控制。
- `pixi install --locked` 与 `pixi run build` 已通过；Clang 21.1.8、CMake 3.31.8、Ceres 2.2.0、Eigen 3.4.0、Python 3.12.15。
- `pixi run start` 已验证配置加载、融合/跟踪/优化线程启动、三路传感器订阅和 SIGINT 正常退出；默认不启动 RViz。
- LLDB 已实际停在 `fusion_ros.cc:250` 的 `main` 源码断点，能显示参数和当前栈帧。
- Debug 构建以 0.5 倍速回放 Urban39 前 30 秒真实数据，完成 GNSS/INS 初始化、视觉跟踪、三角化、Ceres 优化与边缘化；输出 `output/urban39/T20261018220110/trajectory.csv`，226 个位姿、8 列均为有限值、时间戳严格递增；融合节点与静态 TF 节点均在 SIGINT 后正常退出。
- 本次仅做运行级冒烟验证，未进行完整 180 秒回放、精度评估或 RViz 图形验收；Ceres 在多次优化中仍报告既有的 `NO_CONVERGENCE`。
- 用户要求将非标准 `ulong` 统一替换为 `std::uint64_t`；帧、关键帧、地图点、ID 容器及实现均已迁移并编译通过。
- 编译器仍报告 `tracking.cc` 中现有变长数组的 Clang 扩展警告；不影响本次构建，未额外修改跟踪算法。
- macOS 使用说明位于 README 的 1.6 节；Ubuntu 运行指南保留在 `docs/02-ros2-run-guide.md`。本次未修改现有 Linux 导向的 VS Code Python 路径，也未执行 git commit。

## 历史 Linux 验证

- 增量构建通过，`common_nav_core_test`：1/1 通过。
- `urban39_small` 时长 179.995432851 秒，共 19968 条消息；两次 1 倍速播放均正常结束。
- 两次均输出 1726 个严格递增且无 NaN/Inf 的位姿，节点收到 SIGINT 后干净退出。
- 第二次短片段 evo：APE 2.052522 m、约 1 秒平移 RPE 0.461032 m、旋转 RPE 0.232099°。
- 完整 1866.872 秒 Urban39 回放输出 18595 个位姿；最近一次记录的 evo 为 12.794571 m、3.114232 m、0.174006°。

## 当前已知风险

1. 相同输入的两次短跑时间戳完全一致，但原始位置差 RMSE 为 0.708886 m、最大 4.127353 m，说明视觉/多线程管线存在运行间非确定性。
2. 每次短跑有 38 条 `Failed to get right IMU series`；系统会把 GNSS 对齐到保留状态后继续运行，但这是值得继续优化的时序降级路径。
3. Ceres 多次达到迭代上限并报告 `NO_CONVERGENCE`，当前不会中断处理，但会影响精度和运行稳定性的统计判断。
4. `dataset/urban39_small/metadata.yaml` 的 `files[0].message_count` 为 39936，真实值应为 19968；当前 rosbag2 仍可播放。
5. 内置 nav_core 的上游在迁移时没有许可证文件；发布或再分发前必须补充明确的授权与许可证声明。
6. RViz2 仍需要在可用图形桌面的环境中进行最终人工显示验收。

## 建议的下一步

- 优先修正小数据集元数据中的单文件消息数。
- 若继续做精度回归，先固定线程调度和随机源，再重复多次并报告均值、方差和失败率。
- 若要公开发布，先解决 nav_core 许可证问题。
