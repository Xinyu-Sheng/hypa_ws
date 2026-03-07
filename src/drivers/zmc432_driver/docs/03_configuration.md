# 配置说明

本节介绍如何配置轴参数、使能策略，以及简单介绍 EtherCAT 映射。适合需要修改默认行为的用户。

## 轴参数配置

节点启动时默认会对轴 0–3 应用以下参数：

```yaml
default_units: 1.0
default_speed: 10.0
default_accel: 100.0
default_decel: 100.0
```

要更改配置：

1. 修改源码 `src/motion_node.cpp` 中的循环
   ```cpp
   for (int i = 0; i < num_axes; ++i) {
     controller->configure_axis(i, units, speed, accel, decel);
   }
   ```
2. 或者通过扩展添加动态配置服务（TODO）。

单位系统转换公式：
```
脉冲位置 = 物理位置 / units
```
例如 `units=0.001` 时，1 脉冲对应 0.001 mm。

## 电机使能

- 启动后默认轴 0–3 已使能。要更改状态请使用 C++ API 或随包提供的测试程序 (`test_enable`)：
  ```bash
  ros2 run zmc432_driver test_enable
  ```
- 在 `MotionController::queue_motion()` 中，会检查 `axis_enabled` 数组，未使能的轴会被拒绝执行命令。

> **C++ 示例**
> ```cpp
> auto res = controller->enable_axis(0, true);
> if (!res) { /* 错误处理 */ }
> ```

## EtherCAT 总线

本包假设 EtherCAT 主站已在网络中配置，并且 AKD 驱动器映射到连续轴号。启动节点后，如果设置 `perform_ecat_init:=true`，
程序会尝试使用 `libzmotion.so` API 执行总线扫描和 PDO/轴映射。此功能仅限高级用户。

> 参考文档：
> - `ZMC432_V2_User_Manual_v1.6.0.txt`
> - `AKD_EtherCAT_Communications_Manual`。

在大多数简单场景，只需在物理控制器上完成 EtherCAT 配置后运行本节点即可。
