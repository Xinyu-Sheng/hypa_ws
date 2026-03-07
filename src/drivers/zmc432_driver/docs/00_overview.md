# 概览

本节面向想了解包目的和功能的用户。适合第一次接触 `zmc432_driver` 的人。

`zmc432_driver` 是 HYPA 硬件驱动包的一部分，提供 ZMC432 运动控制器的 ROS 2 接口。控制器通过 EtherCAT 总线与 AKD 伺服驱动器通信，允许上层通过 ROS 话题发送运动指令并获取状态。

## 核心功能

- 单轴运动：绝对/相对位置、连续速度（JOG）
- 多轴插补：直线、圆弧、螺旋、椭圆、空间圆弧、连续轨迹
- 电机使能/失能与实时状态反馈
- 轴状态与运动进度监控，含告警、到位等
- 错误检测与处理（通信、轴故障、运动取消）

> **术语说明**
> - **DPOS**：计划位置（desired position），即控制器当前规划的目标位置
> - **MPOS**：反馈位置（measured position），由驱动器返回的实际位置
> - **插补**：按定义的几何路径同步控制多个轴的运动（linear、circular等）。

## 系统架构

```
ROS2 Topic Publisher → MotionTopicNode → MotionController → ZMotionWrapper → ZMC432 → EtherCAT → AKD Drives
ROS2 Topic Subscriber ← MotionTopicNode ← MotionController ← ZMotionWrapper ← ZMC432 ← EtherCAT ← AKD Drives
```

## 相关资源

- 消息定义：`hypa_msgs/msg/MotionCommand.msg` 、`hypa_msgs/msg/MotionStatus.msg`
- 参见 `docs/index.md` 中的其他章节用于快速上手或开发者内容

## 参考文档

- `docs/manuals/ZMC432_V2_User_Manual_v1.6.0.txt` - ZMC432 用户手册
- `docs/manuals/AKD_EtherCAT_Communications_Manual_EN_REV_U.txt` - AKD EtherCAT 通信手册
- `docs/manuals/ZMotion_PC_Library_Programming_Guide_v2.1.2.txt` - ZMotion PC 库编程手册
