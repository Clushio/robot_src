# robot_r 启动编排

`robot_r` 是项目的真机集成包。它不实现主要算法，而是把 MID360s、LIO、地图、
Nav2 兼容层、AutoNAV、Tag 和 RViz2 按实际机器人配置组合起来。

日常操作建议从仓库根目录的 [README](../README.md) 和
[操作手册](../docs/OPERATIONS.md) 开始，不要只根据 launch 文件名前的数字猜启动顺序。

## 推荐启动顺序

```text
CAN
 └─ ranger_bringup/ranger_mini_v2.launch.py
      ├─ cmd_vel_arbiter
      ├─ collision_monitor
      └─ ranger_base_node

robot_r/3startlocation.launch.py
 ├─ 静态 TF
 ├─ MID360s 驱动
 ├─ map_server
 ├─ LIO 定位
 └─ RViz2（可关闭）

robot_r/5nav.launch.py
 └─ mxb_move_base Nav2 编排 + 全局/局部规划和代价地图

robot_r/3navlocations.launch.py
 └─ x2bot_teleop/runnav 拓扑任务服务
```

底盘必须先于导航启动，因为底盘 launch 同时建立完整速度安全链路。只启动
`5nav.launch.py` 不会自动启动底盘、仲裁器或碰撞监控。

## Launch 文件对照

| 文件 | 作用 | 建议 |
|---|---|---|
| `s2lam.launch.py` | MID360s 驱动 + LIO 三维建图 | 当前 GUI 建图入口 |
| `3startlocation.launch.py` | 静态 TF + MID360s + lifecycle map_server + LIO 定位 + 可选 RViz2 | 当前 GUI 定位入口 |
| `5nav.launch.py` | 点云转换、Nav2 兼容层、代价地图、B 样条局部规划 | 当前导航入口 |
| `3navlocations.launch.py` | 启动 `runnav`，提供拓扑任务服务 | Nav2 就绪后启动 |
| `3settinglocation.launch.py` | 启动实车点位记录节点 | 实车采点模式使用 |
| `4genmap.launch.py` | `GlobalMap.pcd` 转二维 `/map` + 可选 RViz2 | 二维地图生成 |
| `6tagReadAndCtl_mm3v.launch.py` | MM3V 串口读取 + TagCtl | 当前 Tag 精调入口 |
| `6tagReadAndCtl_srv.launch.py` | 只启动 TagCtl，不启动读头 | 外部已发布 `/tag_position` 时使用 |
| `robottf.launch.py` | 发布 `body -> base_link -> livox_frame` 静态 TF | 被定位入口包含 |
| `pcd2pgm.launch.py` | 配置 `pcd2topic` | 被 `4genmap.launch.py` 包含 |
| `2slam.launch.py` | 只包含 LIO mapping，不包含 MID360s 驱动 | 单独调试 LIO 时使用 |
| `3location.launch.py` | MID360（非 MID360s）定位组合 | 非默认，使用前核对硬件 |
| `testlocation.launch.py` | MID360 + 可选 MAVROS 的测试组合 | MAVROS 默认关闭，测试入口 |

`launch_ROS1/` 仅作为旧配置参考，不安装到 ROS2 package。当前主流程由
`livox_mid360.launch.py` 使用本包的 `MID360s_config.json` 启动官方
`livox_ros_driver2`。

## 常用命令

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# 建图
ros2 launch robot_r s2lam.launch.py

# 定位，不由 launch 打开 RViz
ros2 launch robot_r 3startlocation.launch.py rviz_enable:=false

# Nav2 导航编排
ros2 launch robot_r 5nav.launch.py

# AutoNAV
ros2 launch robot_r 3navlocations.launch.py

# 二维地图生成
ros2 launch robot_r 4genmap.launch.py

# Tag，可覆盖设备参数
ros2 launch robot_r 6tagReadAndCtl_mm3v.launch.py port:=/dev/ttyUSB0 baudrate:=38400
```

## 地图和定位参数

ROS2 主配置位于 `lio/config/mid360_ros2.yaml`：

- `common`：雷达和 IMU Topic；
- `preprocess`：盲区、距离、角度 FOV；
- `mapping`：噪声、外参、探测范围；
- `dynamic_filter`：在线动态点过滤；
- `pcd_save`：地图保存；
- `load_g_map`、`load_f_map`：定位地图；
- `init_trans`、`init_rpy` 和 ICP 阈值：重定位初值与约束。

LIO 的地图目录应统一为 `~/maps`。当前实现仍可能包含固定绝对 `ROOTDIR`；部署前应
通过 ROS 参数或用户目录展开完成参数化，不要通过创建额外用户目录来兼容固定路径。

## 二维地图参数

`launch/pcd2pgm.launch.py` 控制：

- 输入目录和 `GlobalMap.pcd` 文件名；
- Z 高度范围；
- 半径滤波；
- 体素栅格；
- 二维地图分辨率和 Topic。

高度范围和滤波会直接影响低矮障碍物是否进入二维地图。修改后必须在 RViz 与实物
对照验证。

## TF 约定

默认静态关系：

```text
body --(-0.33 m)--> base_link --(+0.33 m)--> livox_frame
```

数值在 `launch/robottf.launch.py`。修改雷达安装位置后必须同步标定 LIO 外参、传感器
frame 和机器人 footprint，不能只修改 RViz 显示。

## RViz2

- `rviz/nav_rviz.rviz`：带项目 Panel 的导航视图。
- `rviz/nav_rviz_no_panel.rviz`：GUI 独立运行时使用的普通导航视图。
- `rviz/mapgen_rviz.rviz`：二维地图生成视图。
- `lio/launch/include/rviz.rviz`：三维建图视图。

RViz2 中“记录点位”工具由本包插件提供，发布到 `/anav/record_pose`。

## 修改 launch 时的检查项

- 是否重复启动同名雷达、RViz、map_server 或底盘节点；
- 是否仍由底盘 launch 包含仲裁器和碰撞监控；
- map、odom、base_link、livox_frame 的 TF 是否闭合；
- 参数 namespace 是否与 Nav2 节点中读取的一致；
- GUI 中对应按钮的命令是否同步更新；
- README 和操作手册是否需要同步。
