# IC-GVINS 当前状态

最后更新：2026-10-18

当前任务：按用户确认的简化方案，将 nav_core 的必要实现并入 common，删除 `cmake/`、`third_party/` 和 `test/`；完成构建验证后更新本状态。

过程记录：首次中间构建时，旧兼容测试因 common 已切断对 `syscoord.hpp` 的间接包含而找不到 `nav::` 参考符号；测试已临时改为显式包含旧参考头，仅用于删除前的新旧数值对照。

第二次中间构建时，静态链接器因核心库已不再使用 NavTime 而没有带入旧时间符号；临时测试目标显式链接旧 `open_nav_core` 后继续对照，该目标和链接项将在验证后一起删除。

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

## 最近验证

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
