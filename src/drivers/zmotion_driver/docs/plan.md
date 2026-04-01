# zmotion_driver 力矩回读与 joint_states 改造计划

## 背景

当前 `zmotion_driver` 已经能够发布 `/joint_states`，其中 `position` 和 `velocity` 分别来自 ZMotion PC 函数库的 `GetMpos()` / `GetMspeed()`，但 `effort` 目前固定填充为 `0.0`。ZMotion 文档说明，读取当前总线驱动器力矩需要先配置正确的 `ATYPE` 和 `DRIVE_PROFILE`，然后再调用对应的力矩读取接口。

## 目标

1. 让 `/joint_states.effort` 输出真实力矩值，而不是固定 0。
2. 明确当前启动参数下的 `DRIVE_PROFILE` 行为，避免把 `DRIVE_MODE` 与 `DRIVE_PROFILE` 混淆。
3. 保证改造后节点仍符合 Lifecycle 流程，并能稳定通过 `configure` / `activate`。

## 实施步骤

### 1. 确认力矩读取接口

- 在 `zmotion_sdk_wrapper` 中确认并封装力矩读取能力。
- 优先使用 ZMotion PC 函数库中已有的总线力矩接口。
- 明确返回单位、有效范围和失败处理方式。

### 2. 修改 `/joint_states` 发布逻辑

- 在反馈发布路径中增加力矩读取。
- 将读取到的力矩写入 `sensor_msgs::msg::JointState::effort`。
- 如果单轴读取失败，保留当前行为的容错策略：记录日志，并避免阻断其他轴的数据发布。

### 3. 明确 PDO / 模式配置

- 检查当前 `ecat.init.drive_pdo_mode` 的默认值和生效方式。
- 区分以下概念：
  - `DRIVE_PROFILE`：PDO 配置
  - `DRIVE_MODE`：驱动器控制模式，对应 0x6060
- 根据驱动器型号和手册，确认哪种 `DRIVE_PROFILE` 组合支持力矩回读。

### 4. 验证启动参数

- 用当前启动方式复现实机行为：

```bash
ros2 run zmotion_driver zmotion_driver_node --ros-args -p controller.ip:=192.168.0.11 -p ecat.init.drive_axis_num:=-1
```

- 检查 `/joint_states` 中的 `position`、`velocity`、`effort` 是否符合预期。
- 检查 lifecycle 状态切换时是否仍能正常连接、激活和停止。

### 5. 补充说明文档

- 在 `docs/lifecycle_switch.md` 或新文档中补充：
  - 力矩回读前置条件
  - `DRIVE_PROFILE` 取值含义
  - 当前默认配置的实际效果

## 验收标准

- `/joint_states.effort` 不再固定为 `0.0`。
- 力矩读取失败时，节点不会整体失效。
- 文档中能明确区分 `DRIVE_MODE` 和 `DRIVE_PROFILE`。
- 按当前启动命令运行后，生命周期切换和反馈发布都正常。

## 风险与注意事项

- 不同驱动器型号对 PDO 配置支持不同，默认 `DRIVE_PROFILE` 不一定包含力矩反馈。
- 力矩值可能需要按驱动器文档做单位换算，不能直接假设和位置单位一致。
- 若只修改发布逻辑但不调整 PDO 配置，`effort` 可能仍然读不到有效值。
