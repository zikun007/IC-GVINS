# IC-GVINS 坐标系与位姿约定

本文记录当前 common 模块实际采用的坐标系和变量方向：

```text
R_a_b：把 b 系向量旋转到 a 系，v_a = R_a_b * v_b
t_a_b：b 系原点在 a 系中的坐标，p_a = R_a_b * p_b + t_a_b
T_a_b = [R_a_b, t_a_b]
```

## 坐标系

| 符号 | 含义 | 轴向 |
|---|---|---|
| E | 地心地固 ECEF | WGS84 ECEF |
| N | 当前位置的地理导航系 | NED：北、东、地 |
| W | 初始化后固定的局部世界系 | 与首个 GNSS 点的 NED 对齐 |
| B | IMU/机体系 | FRD：前、右、下 |
| C | OpenCV 相机光学系 | RDF：右、下、前 |
| V | KAIST 车辆系 | FLU：前、左、上 |

`N` 随经纬度改变，`W` 在一次解算中保持不动。两者只在初始化原点处重合，不能在远距离轨迹上混为同一个坐标系。

## 强类型位姿

旧 `Pose {R, t}` 同时表示多种不兼容语义，现已拆分为：

| 类型 | 旋转字段 | 位置字段 | 严格含义 |
|---|---|---|---|
| `LocalBodyPose` | `R_w_b` | `t_w_b` | `T_w_b`，机体 B 到固定世界 W |
| `GlobalBodyPose` | `R_n_b` | `blh_b` | B 到当前位置 N 的姿态，加机体的经纬高；不是刚体变换 |
| `CameraPose` | `R_w_c` | `t_w_c` | `T_w_c`，相机 C 到固定世界 W |
| `CameraExtrinsics` | `R_b_c` | `t_b_c` | `T_b_c`，相机 C 到 IMU 机体 B |

优化状态 `IntegrationState` 的既有字段暂时保持二进制布局：`p = t_w_b`、`q = q_w_b`、`v = v_w_b`。优化参数数组同样按 `[t_w_b, q_w_b_xyzw]` 排列。

## 实际变换链

### GNSS global → local

GNSS 输入为 `blh_g = [lat, lon, h]`，先转到 ECEF，再投影到固定原点 NED：

```text
t_w_g = R_w_e * (t_e_g - t_e_w0)
```

GNSS 因子用 B 系杆臂 `l_b_g` 约束天线位置：

```text
t_w_g(predicted) = t_w_b + R_w_b * l_b_g
```

这与 `nav::WorldFrame::blh2world()`、`nav::sensor2body_pos()` 的方向一致。

### INS/body → camera → pixel

相机外参由配置提供 `T_b_c`，相机世界位姿为：

```text
R_w_c = R_w_b * R_b_c
t_w_c = t_w_b + R_w_b * t_b_c
```

世界点投影到相机和像素：

```text
p_c = R_w_c^T * (p_w - t_w_c)
u = (fx * x_c + skew * y_c) / z_c + cx
v = fy * y_c / z_c + cy
```

因此 `z_c > 0` 表示点位于相机前方，`u` 向右、`v` 向下，正是 RDF/OpenCV 约定。重投影因子采用相同的 `C0 → B0 → W → B1 → C1` 链。

### local body → global body

输出全球姿态时：

```text
blh_b = world2blh(t_w_b)
R_n_b = R_n_w(blh_b) * R_w_b
```

这里必须使用当前位置的 `N`，不能直接把 `R_w_b` 当作 `R_n_b`。

## KAIST urban39 对应关系

KAIST 标定文件的 `Vehicle2Stereo.txt` 给出 `T_v_c`，`Vehicle2IMU.txt` 给出 `T_v_i`。配置生成脚本计算：

```text
R_b_c = R_b_v * R_v_c
t_b_c = R_b_v * (t_v_c - t_v_i)
R_b_v = diag(1, -1, -1) * R_i_v
```

rosbag 生成脚本也用 `diag(1, -1, -1)` 把 IMU 测量从 KAIST FLU 转成算法需要的 FRD。Urban39 当前配置的 `t_b_c` 与标定推导值误差小于 `1e-16 m`，GNSS 杆臂同样一致；`R_b_c` 的列分别接近 B 系的右、下、前三个方向。

## 与精度问题的关系

当前检查没有发现 local/global 的 NED/ENU 翻转，也没有发现 `T_b_c` 被当成 `T_c_b`。合并前的回归测试确认 `global2local`、姿态与时间转换和原 nav_core 实现一致；相机和像素代码保持 OpenCV RDF 约定。

因此，坐标系方向错误目前不是完整回放精度波动的首要嫌疑。更需要优先隔离的是已有三线程处理顺序造成的运行间非确定性。需要注意：RViz 通常按 Z-up 理解世界，而算法发布的是 NED/Z-down；这会影响画面朝向，但不会改变估计器内部精度。
