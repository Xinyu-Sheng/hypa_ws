# zmotion_driver 架构说明与安全评估

## 1. 概述

`zmotion_driver` 是一个 ROS 2 Lifecycle 驱动节点，位于包 `zmotion_driver` 中。它的核心职责是：

- 与 ZMotion 控制器建立 Ethernet 连接
- 通过 SDK 初始化 EtherCAT 现场总线
- 配置和管理多轴伺服驱动器
- 接收 ROS 2 控制命令并下发到硬件
- 周期性采集轴位置、速度、扭矩以及 IO 状态
- 提供 emergency stop、安全停止和生命周期恢复机制

该包的核心实现分为两层：

- `ZMotionDriverNode`：Lifecycle 节点、参数读取、话题/订阅、定时器、控制逻辑
- `ZMotionSdkWrapper`：对第三方 ZMotion SDK 的封装，负责底层连接、命令调用和读写接口

## 2. 包结构与职责边界

### 2.1 关键文件

- `src/zmotion_driver_node.cpp`
  - Lifecycle 生命周期管理
  - 参数加载与验证
  - ROS 话题/订阅、定时器创建
  - command dispatch、feedback 采集、IO 轮询、安全 stop

- `include/zmotion_driver/zmotion_driver_node.hpp`
  - `ZMotionDriverNode` 的 lifecycle 接口声明

- `src/zmotion_sdk_wrapper.cpp`
  - SDK 接口封装
  - 动作命令、轴配置、输入读取、状态读取

- `include/zmotion_driver/zmotion_sdk_wrapper.hpp`
  - `ZMotionSdkWrapper` 公共接口

- `config/zmotion_driver.yaml`
  - 该节点的参数配置模板
  - 定义 controller、axis、control、feedback、io、ecat.init 等配置

- `launch/zmotion_driver.launch.py`
  - LifecycleNode 启动方式
  - 支持 namespace、use_sim_time、参数文件

- `docs/lifecycle_switch.md`
  - 运行时生命周期切换说明

### 2.2 逻辑层次

- 参数层：读取 ROS 参数并验证数组长度、逻辑轴与 mimic 组约束
- 设备初始化层：连接控制器、初始化 EtherCAT、配置各轴并设置轴使能状态
- 接口创建层：发布 `joint_states`、轴位置 topic、IO 状态 topic；订阅速度命令、mimic 命令、制动命令；创建两个周期性定时器
- 执行层：命令映射、硬件 buffer 检查、SDK 下发命令、命令日志记录
- 反馈层：定时读取位置/速度/扭矩并发布 joint_state 和 axis position；按 group 发布 mimic 位置
- IO 触发层：轮询输入、状态发布、边沿/电平触发、IO 命令映射到速度控制
- 安全层：`emergency_stop` 锁定、`StopAll()`、`SetAxisEnable(false)`、lifecycle 恢复需求

## 3. Lifecycle 运行流程

### 3.1 on_configure

`on_configure()` 依次执行：

1. 读取并验证参数 `LoadParameters()`
2. 打开驱动日志文件 `OpenLogFile()`
3. 打开 joint state CSV 日志 `OpenJointStateFile()`
4. 打开 command CSV 日志 `OpenCommandFile()`
5. 硬件初始化 `ConfigureHardware()`
6. 创建 ROS 接口 `CreateInterfaces()`

如果任一步失败，则调用 `TearDown()` 并返回 `FAILURE`。

### 3.2 on_activate

`ActivateNode()` 会：

- 如果 `enable_axis_on_activate=true` 且未处于 emergency stop，则对所有轴调用 `SetAxisEnable(true)`
- 将内部标记 `active=true`
- 激活发布者 `joint_state_pub`、各 axis position publisher、mimic position publisher、IO 状态 publisher

### 3.3 on_deactivate

`DeactivateNode()` 会：

- 将 `active=false`、`emergency_stop=false`
- 调用 `StopAll()`
- 对所有轴调用 `SetAxisEnable(false)`
- 取消激活所有 lifecycle 发布者

### 3.4 on_cleanup / on_shutdown

- `on_cleanup()` 调用 `CleanupNode()`，其实质是 `TearDown()`
- `on_shutdown()` 也直接调用 `TearDown()`

`TearDown()` 内部会断开 SDK、重置接口、关闭日志文件、关闭 CSV 文件

## 4. 参数与轴配置策略

### 4.1 逻辑轴与物理轴

代码中定义 `kAxisCount = 12`，表示固定位数的物理轴配置。运行时：

- `axis.logical_indices` 规定每个物理轴对应的逻辑轴编号
- 对应 `axis.joint_names`、`axis.position_modes`、`axis.zero_offsets`、`axis.units`、`axis.directions`、`axis.speeds`、`axis.accels`、`axis.decels`、`axis.position_topics`
- 控制逻辑将基于 logical index 查找对应 physical axis

### 4.2 控制与 mimic 分组

- `control.velocity_logical_indices` 定义速度控制轴集合
- `control.mimic_group1_logical_indices`、`control.mimic_group2_logical_indices` 定义两个 mimic 位置控制组
- 代码要求：velocity 组必须 4 个轴，mimic 组每组必须 4 个轴，且两个 mimic 组不允许重叠，velocity 组与 mimic 组也不得重叠

### 4.3 模式与命令映射

- `axis.position_modes` 支持 `absolute` / `relative`
- `OnMimicCommand()` 会
  - if position_mode == absolute -> `MoveAbsoluteMulti`
  - else -> `MoveRelativeMulti`
- `OnVelocityCommand()` 将 `Float64MultiArray` 数据按顺序下发给 `CommandVelocity`
- 轴方向 `axis.directions` 用于对控制值与反馈值做符号修正
- 绝对位置命令会加上 `zero_offset`，反馈位置会减去 `zero_offset`

### 4.4 IO 配置

- `io.input_ids` 与 `io.state_topics` 一一对应
- `io.emergency_stop_on_high` 用于兼容旧逻辑
- `io.trigger_modes` 支持：`none/level_high/level_low/rising_edge/falling_edge/both_edges`
- 如果未提供 `io.trigger_modes`，且 `io.emergency_stop_on_high=true`，则回退为 `level_high`

## 5. SDK 封装层行为

### 5.1 连接与断开

- `ZMotionSdkWrapper::Connect()` 使用 `ZAux_OpenEth()` 连接 Ethernet 控制器
- 仅在未连接时才执行连接逻辑
- `Disconnect()` 调用 `ZAux_Close()` 并清空状态
- 析构函数中自动调用 `Disconnect()`

### 5.2 EtherCAT 初始化

- `InitEthercat()` 在控制器已连接情况下执行
- `ConfigureHardware()` 会循环所有轴执行 `ConfigureAxis()` 并将轴预置为未使能状态

### 5.3 运动命令与停止

- 速度命令：`CommandVelocity(axis, velocity)`
- 位置命令：`MoveAbsoluteMulti()` / `MoveRelativeMulti()`
- 停止命令：`CancelAxis()` 和 `StopAll()`

### 5.4 反馈与状态读取

- `GetMpos()`、`GetMspeed()`、`GetDriveTorque()` 读取物理轴状态
- `GetInput()` 读取 IO 输入
- `GetRemainBuffer()` 用于命令发送前的硬件缓冲判断

## 6. 控制命令流与安全链路

该节点在命令执行前具备几项关键安全检查：

1. `DispatchCommandLocked()` 之前通过 `configured && active && !emergency_stop` 过滤非激活或 emergency stop 状态
2. 逻辑轴映射失败会直接拒绝执行
3. 命令值数量必须与 axis 数量匹配
4. 对每个目标物理轴读取 `GetRemainBuffer()` 并与 `min_remain_buffer` 比较
5. `ExecuteCommandLocked()` 执行失败时触发 emergency stop

### 6.1 emergency stop 机制

- 任何硬件缓冲检查失败、命令执行失败、IO 安全输入触发、`brake_topic` 发送 `true`，都会调用 `TriggerEmergencyStopLocked()`
- `TriggerEmergencyStopLocked()` 会：
  - 设 `emergency_stop=true`
  - 调用 `StopAll()`
  - 将所有轴使能置为 `false`
- emergency stop 后，节点保持锁定状态，后续命令均不可执行
- `brake_topic=false` 只能输出警告，不会自动清除 emergency stop，必须通过生命周期重复 `configure`/`activate` 恢复

### 6.2 IO 触发命令

- IO 输入按周期 `io_period_ms` 轮询
- 每次读取触发 `DispatchIoTriggerLocked()`，并发布 IO 状态 topic
- IO 触发可将 IO 输入映射为：
  - 轴速度控制（IO 0-7 对应逻辑轴 4-7）
  - mimic group 速度控制（IO 8-11 对应 mimic 组 1/2）
- 此外，`emergency_stop_on_high=true` 的 IO 会在高电平时立即进入 emergency stop

## 7. 关键安全建议

### 7.1 `enable_axis_on_activate` 的安全边界

- 当前设计在 `activate()` 时直接使能所有轴，若此时底层硬件状态异常，则可能产生未预期运动
- 建议增加：
  - axis enable 前的硬件自检
  - enable 失败时回退到 `deactivate` 或 `cleanup`
  - 支持按组或按轴使能，避免单点异常导致全轴失控

### 7.2 emergency stop 状态恢复

- `brake_topic=false` 不恢复 emergency stop，当前只能通过生命周期重载恢复
- 建议明确：
  - emergency stop 是“latched”安全状态，必须上下文外手动恢复
  - 提供一个安全恢复 topic 或 service，避免运维误解

### 7.3 `StopAll()` / `SetAxisEnable(false)` 的错误处理

- 目前 `StopAll()` 和 `SetAxisEnable(false)` 的结果值被忽略，只记录 `void` 返回值
- 建议至少记录失败原因，并在 `cleanup`/`deactivate` 中实现重试或告警
- 对于 `StopAll()` 失败，应保留 `emergency_stop=true` 并阻止后续激活

### 7.4 IO 轮询与触发逻辑风险

- IO 触发是在 `mutex` 内同步执行的，若 SDK 调用阻塞或失败，会阻塞整个 `PollIoInputs()` 轮询
- 建议将 IO 状态读取与动作触发拆分：
  - 读取阶段仅读取并发布状态
  - 触发阶段在单独线程/任务中执行，避免轮询延迟影响系统
- `io_debounce_ms` 与 `log_on_change` 只影响日志，不影响触发逻辑；若 IO 可靠性差，需保证触发安全而非仅日志稳定

### 7.5 参数配置校验增强

- 参数数组长度已验证，但未检测具体数值范围，例如：
  - `axis.directions` 只能为 `{1,-1}`，已校验
  - 建议增加：速度、加速度、减速度、units、zero_offsets 的物理合理性范围约束
  - `control.velocity_logical_indices` 与 mimic 组是否确实对应需要额外的运行时打印检查
- `ecat.init.drive_axis_num` 在 YAML 中可设 `-1`，这在正式部署时很危险，建议明确为固定轴数 12 或根据硬件限制校验

### 7.6 日志与文件 I/O 的鲁棒性

- 当前日志文件路径会追加时间戳并创建目录，若磁盘不可写则配置失败
- 建议增加：
  - 控制路径是否为空的默认处理
  - 日志打开失败时 fallback 到 `/tmp` 或禁用 CSV 记录模式
  - 记录失败写入时的异常，但不影响驱动核心功能

## 8. 典型功能疑问点

### 8.1 `axis.position_topics` 对 mimic 轴的处理

- 代码中对 `axis_position_pubs` 仅为非 mimic 成员创建 publisher
- 但 `axis.position_topics` 仍要求长度为 12，并且为所有轴提供 topic 名称
- 疑问：对于 mimic 组轴，`position_topics` 是否只是占位，并不实际发布？如果是，应在参数文档中明确

### 8.2 `ecat.init.drive_axis_num` 可为 `-1`

- YAML 中默认值 `-1` 表示不限制节点数量
- 但这种配置对 EtherCAT 总线初始化、安全性和硬件一致性不是最佳实践
- 应核实：实际 ZMotion SDK 是否允许 `-1`，以及在 12 轴系统中是否应强制为 `12`

### 8.3 `OnMimicCommand()` 的 position_mode 语义

- `mimic_group` 输入仅接收单个 `Float64`，在绝对模式下直接调用 `MoveAbsoluteMulti`，在相对模式下调用 `MoveRelativeMulti`
- 疑问：这是否意味着所有 mimic 组轴必须以相同目标值移动？对于 4 轴 mimic 组是否存在偏移或单轴差异需求？

### 8.4 emergency stop 与 `brake_topic` 的关系

- `cmd/brake` 为 `true` 时直接触发 emergency stop，但 `false` 仅记录警告
- 这会让操作人员误以为 `false` 可解除制动，实际上必须重新配置生命周期才能恢复
- 需要在用户文档中明确这一点，避免误操作

### 8.5 IO 输入编号与逻辑轴映射固定性

- 当前实现将 IO 0-7 固定映射到逻辑轴 `4..7` 和 `8..11` 中的 `IO_LOGICAL_AXIS_OFFSET` 计算结果
- 这一映射没有从参数中配置，属于隐式设计
- 建议确认这是否为固定硬件映射，还是应支持可配置 IO 对应关系

### 8.6 `GetRemainBuffer()` 安全阈值效果

- `CheckHardwareBufferLocked()` 只要任意轴的 remain buffer 小于等于 `min_remain_buffer` 就触发 emergency stop
- 这是一条强安全规则，但如果读取失败也会被视为失败并触发 emergency stop
- 建议确认 buffer 阈值是否应按轴、按命令类型区分，或是否允许在 emergency stop 后用更低频率再次尝试恢复

## 9. 结论

`zmotion_driver` 的架构总体上符合 ROS 2 Lifecycle 驱动的设计：

- 清晰分离 lifecycle 节点逻辑与 SDK 封装
- 支持参数化的逻辑轴映射与 mimic 组
- 有 emergency stop 机制并结合 hardware buffer 检查

但从安全性角度看，当前实现存在几处隐含风险：

- emergency stop 恢复路径不明确
- `StopAll()`/`SetAxisEnable(false)` 失败未充分处理
- IO 触发/轮询逻辑与 SDK 阻塞耦合
- 部分配置依赖隐式映射，不够透明

建议将上述风险点补充到设计说明与使用文档中，并在未来版本中补强恢复机制、错误上报和参数范围校验。
