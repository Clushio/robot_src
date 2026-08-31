# anav_ranger

ROS2 Humble 的 Ranger AutoNAV 操作界面和部署工具包。

推荐入口：

```bash
ros2 run anav_ranger aNAV_ranger.py
```

主界面保留建图、定位、点位采集、拓扑生成、Nav2/AutoNAV、Tag、软件急停、
诊断和底盘启动流程。ROS1 的 master 探测改为 ROS2 上下文与 graph 探测；所有
`roslaunch/rosservice/rostopic/rosrun` 调用改为对应的 `ros2` 命令。

`mm3v_serial_reader.py` 和 `tcpserver.py` 是兼容入口，实际调用 `robot_r` 中已经过
测试的单一实现，避免两份串口协议或 TCP 服务逻辑漂移。

`stpnav_B.py`、`stpnav_C.py`、`stpnav_D.py` 是历史简化界面。ROS1 原工程没有它们
引用的 `*_C.launch`、`*_D.launch` 文件，因此 ROS2 中三者均映射到当前可用的
`3startlocation.launch.py`、`5nav.launch.py` 和 `3navlocations.launch.py`；这是
无法直接一一映射时的最小可运行方案，不改变导航算法。

`pathRecord.py` 是实验轨迹对比工具，ROS2 使用 `ros2 bag record/play` 和 RViz2。
其原硬编码 `/home/jgl20/testrecord` 改为可移植的 `~/testrecord`。
