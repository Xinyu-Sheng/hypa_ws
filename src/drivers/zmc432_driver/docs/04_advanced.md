# 高级功能

本节面向需要定制 EtherCAT 初始化、处理复杂错误或连续轨迹的高级用户。

> ⚠️ 备注：底层 C API `ZAux_BusCmd_EcatScan` 已重命名为
> `ZAux_BusCmd_EcatInit`，C++ 封装对应提供了 `ecat_init()` 方法。
> 这是一个破坏性改动，旧名称已移除。

## EtherCAT 初始化（ECAT init）

可以在节点启动时通过参数或程序调用执行总线初始化。

**C++ API**
```cpp
#include "zmc432_driver/ecat_init.hpp"
#include "zmc432_driver/motion_controller.hpp"

zmc432_driver::EcatInitInfo info; // 默认参数
auto err = controller->initialize_bus(info, /*slot*/0, /*timeout_ms*/5000);
if (err) { /* 处理错误 */ }
```

`EcatInitInfo` 对应底层 `EcatInitInfoSet`，高级用户可修改字段后调用 `info.toC()`。

*新的行为*：`initialize_bus()` 调用会在扫描成功后自动执行总线轴数累加、轴地址/类型映射、PDO 模式设置、IO 映射以及总线启动和报警清除。
`EcatInitInfo` 中的 `drive_pdo_mode`、`drive_io_stara`、`drive_io_spa`、
`node_io_id`/`node_aio_id` 以及 `drive_enable` 字段均用于这些步骤。
如果 `drive_axis_num` 或 `ecat_node_num` 被设置为非负值，函数还会
与实际扫描结果进行验证并在不匹配时返回错误。

上述自动化逻辑是从原 Windows 示例移植而来，默认使用
`use_defaults=true` 时与旧版本行为兼容，只做简单扫描。
**启动参数**
```yaml
perform_ecat_init: true
ecat_slot_id: 0
ecat_timeout_ms: 5000
ecat.drive_axis_start: 0
ecat.drive_axis_num: -1
``` 

## 错误处理

- **通信错误**：控制器断连时节点会尝试重连，并在 `motion_status.error_message` 中报告。
- **轴故障**：AKD 紧急消息会转换为状态字并转发到 `axis_statuses`。
- **运动取消**：可发布空 `MotionCommand` 或在 `MotionController` 调用 `cancel_motion()`。

## 连续轨迹流式缓冲

支持通过多点命令连续发送轨迹，适用于点胶、焊接等应用。设置 `motion_type=3` 并在 `positions` 中附加后续点，节点会在内部维护缓冲。

## 未来扩展（开发计划）

- 添加轴配置服务（动态修改）
- 回零（homing）动作支持
- 位置/速度/力矩模式切换
- 插补参数验证与预处理
- 多控制器冗余
