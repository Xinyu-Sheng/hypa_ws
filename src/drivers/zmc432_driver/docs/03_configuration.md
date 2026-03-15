# 配置说明

本节介绍如何配置轴参数、使能策略，以及简单介绍 EtherCAT 映射。适合需要修改默认行为的用户。

## 轴参数配置

节点启动时会读取以下参数：

* `axis_count`：轴的数量，默认 1；
* `default_units`、`default_speed`、`default_accel`、`default_decel`：默认的单位换算、速度、加速度、减速度。

下面是一个 YAML 示例：

```yaml
axis_count: 6
default_units: 0.001
default_speed: 20.0
default_accel: 200.0
default_decel: 200.0
```

在 C++ 节点中可以这样声明和读取参数：

```cpp
int axis_count;
node->declare_parameter("axis_count", 4);
node->get_parameter("axis_count", axis_count);

double units, speed, accel, decel;
node->declare_parameter("default_units", 1.0);
node->declare_parameter("default_speed", 10.0);
node->declare_parameter("default_accel", 100.0);
node->declare_parameter("default_decel", 100.0);
node->get_parameter("default_units", units);
node->get_parameter("default_speed", speed);
node->get_parameter("default_accel", accel);
node->get_parameter("default_decel", decel);

for (int i = 0; i < axis_count; ++i) {
  controller->configure_axis(i, units, speed, accel, decel);
}
```

也可以在 launch 文件中设置参数：

```xml
<node pkg="zmc432_driver" exec="motion_node" name="motion" output="screen">
  <param name="axis_count" value="8"/>
  <param name="default_units" value="0.002"/>
</node>
```

或者使用命令行工具：

```bash
ros2 param set /motion axis_count 6
ros2 param set /motion default_units 0.005
```

这些设置会在节点启动时应用于每个轴的默认配置，修改后需要重启节点才能生效。

单位系统转换公式：
```
脉冲位置 = 物理位置 / units
```
例如 `units=0.001` 时，1 脉冲对应 0.001 mm。


## EtherCAT 总线

本包假设 EtherCAT 主站已在网络中配置，并且 AKD 驱动器映射到连续轴号。启动节点后，如果设置 `perform_ecat_init:=true`（可通过 launch 参数 `perform_ecat_init` 或 `ros2 param set` 来设置），程序会尝试使用 `libzmotion.so` API执行总线扫描和 PDO/轴映射。此功能仅限高级用户。

> 参考文档：
> - `ZMC432_V2_User_Manual_v1.6.0.txt`
> - `AKD_EtherCAT_Communications_Manual`。

在大多数简单场景，只需在物理控制器上完成 EtherCAT 配置后运行本节点即可。
