# 文档索引

`docs/` 只保存面向项目使用者的正式文档，以及尚待确认的设计方案。

## 正式文档

- [02-ros2-run-guide.md](02-ros2-run-guide.md)：ROS2 Jazzy 依赖、构建、Cyclone DDS、启动、Urban39 回放和 evo 评测。
- [03-coordinate-frames.md](03-coordinate-frames.md)：W/N/E/B/C/V 坐标系、强类型位姿和相机投影约定。

## 文档管理约定

- 已确认的项目方案归档到 `.agents/project/`。
- 实施进度、调试结论和当前风险归档到 `.agents/memory/`。
- 新的待确认方案放在本目录，使用下一个可用的两位序号和 `<序号>-<description>.md` 格式。
- 方案获得确认后，提炼有效内容到 `.agents/project/`，不在 `docs/` 长期保留重复的 planning 日志。
