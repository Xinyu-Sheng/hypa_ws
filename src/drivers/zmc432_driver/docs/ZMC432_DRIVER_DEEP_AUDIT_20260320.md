# ZMC432 驱动深度审计报告（2026-03-20，已修复版）

## 1. 审计目标
本报告用于审计 zmc432_driver 当前实现是否满足以下要求：
- 功能逻辑完整：配置、初始化、运行、停机全链路可用。
- 与 ZMotion 手册/PC 函数库一致：指令名、参数语义、状态机顺序一致。
- ROS 2 生命周期规范：Lifecycle 状态可正确迁移。
- 代码规范可维护：符合工作区约束与项目规范。

审计对象：
- 驱动代码：[src/drivers/zmc432_driver/src](src/drivers/zmc432_driver/src)
- 头文件：[src/drivers/zmc432_driver/include/zmc432_driver](src/drivers/zmc432_driver/include/zmc432_driver)
- 配置与启动：[src/drivers/zmc432_driver/config/zmc432_params.yaml](src/drivers/zmc432_driver/config/zmc432_params.yaml), [src/drivers/zmc432_driver/launch/motion_server.launch.py](src/drivers/zmc432_driver/launch/motion_server.launch.py)
- 消息定义：[src/hypa_msgs/msg/MotionCommand.msg](src/hypa_msgs/msg/MotionCommand.msg), [src/hypa_msgs/msg/MotionStatus.msg](src/hypa_msgs/msg/MotionStatus.msg)
- 参考文档：[src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt](src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt), [src/drivers/zmc432_driver/docs/txt/ZMotion_PC_Library_Programming_Guide_v2.1.2.txt](src/drivers/zmc432_driver/docs/txt/ZMotion_PC_Library_Programming_Guide_v2.1.2.txt)

ROS 版本确认：`ROS_DISTRO=humble`。

---

## 2. 总体结论
当前实现存在多项高风险问题，尤其集中在：
- Lifecycle 流程不可重激活。
- 运动命令字符串与手册/SDK不一致。
- 单位换算重复导致位移量错误。
- EtherCAT 参数默认机制覆盖用户配置。

本次代码修复后，已关闭 3 项，待关闭 10 项。

建议先完成剩余严重和高等级问题后，再进行真机联调。

### 2.1 闭环状态总览

| 编号 | 状态   | 说明                                 |
| ---- | ------ | ------------------------------------ |
| S1   | 已关闭 | Lifecycle 激活/停用/关机顺序已修复   |
| S2   | 已关闭 | 多轴线性插补改为 SDK 接口            |
| S3   | 已关闭 | 螺旋/椭圆/球面插补改为 SDK 对应接口  |
| S4   | 待关闭 | UNITS 与 pulse 语义仍需统一          |
| S5   | 待关闭 | 抱闸 IO 仍为临时映射                 |
| H1   | 待关闭 | AXIS_ADDRESS 的 slot 高位映射未修复  |
| H2   | 待关闭 | use_defaults 语义覆盖问题未修复      |
| H3   | 待关闭 | current_executing_cmd 并发读写未加锁 |
| M1   | 待关闭 | progress 算法仍未使用起始位置快照    |
| M2   | 待关闭 | wait_time 仍未执行                   |
| M3   | 待关闭 | 代码规范偏差未系统清理               |
| L1   | 待关闭 | CMake 重复 install 未清理            |
| L2   | 待关闭 | 节点参数自包含要求未完全满足         |

---

## 3. 详细问题清单（按严重级别）

## 严重

### S1. Lifecycle 在一次停用后不可恢复，关机位置保存逻辑失效
- 状态：已关闭
- 定位：
  - [src/drivers/zmc432_driver/src/motion_controller.cpp#L308](src/drivers/zmc432_driver/src/motion_controller.cpp#L308)
  - [src/drivers/zmc432_driver/src/motion_controller.cpp#L330](src/drivers/zmc432_driver/src/motion_controller.cpp#L330)
  - [src/drivers/zmc432_driver/src/motion_node.cpp#L324](src/drivers/zmc432_driver/src/motion_node.cpp#L324)
  - [src/drivers/zmc432_driver/src/motion_node.cpp#L332](src/drivers/zmc432_driver/src/motion_node.cpp#L332)
  - [src/drivers/zmc432_driver/src/motion_node.cpp#L300](src/drivers/zmc432_driver/src/motion_node.cpp#L300)
  - [src/drivers/zmc432_driver/src/motion_node.cpp#L310](src/drivers/zmc432_driver/src/motion_node.cpp#L310)
  - [src/drivers/zmc432_driver/src/motion_node.cpp#L356](src/drivers/zmc432_driver/src/motion_node.cpp#L356)
  - [src/drivers/zmc432_driver/src/motion_node.cpp#L364](src/drivers/zmc432_driver/src/motion_node.cpp#L364)
- 问题：
  - `on_deactivate()` 调用 `controller_->stop()` 后断开硬件连接；`on_activate()` 仅 `start()`，不会重连，激活失败。
  - `on_deactivate()` 关闭 `topic_node_`，`on_activate()` 未重新 `initialize()`。
  - `on_shutdown()` 先 `stop()` 再读位置，读位置信息时已断连。
- 影响：Lifecycle 不能正常循环，位置记忆功能实质失效。

### S2. 多轴 MOVE 命令格式与手册/SDK不一致
- 状态：已关闭
- 定位：
  - [src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L475](src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L475)
  - [src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L517](src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L517)
  - [src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L1930](src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L1930)
  - [src/drivers/zmc432_driver/src/zmcaux.cpp#L5482](src/drivers/zmc432_driver/src/zmcaux.cpp#L5482)
- 问题：
  - 代码拼接 `MOVEABS(axis,pos,axis,pos...)`。
  - 文档与 SDK 封装应为 `BASE(...)` 后 `MOVEABS(pos,...)`。
- 影响：多轴插补可能语法错误或行为偏差。

### S3. 多个插补命令名与手册定义不一致
- 状态：已关闭
- 定位：
  - [src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L630](src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L630)
  - [src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L700](src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L700)
  - [src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L765](src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L765)
  - [src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L2617](src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L2617)
  - [src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L2725](src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L2725)
  - [src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L2909](src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L2909)
  - [src/drivers/zmc432_driver/src/zmcaux.cpp#L6876](src/drivers/zmc432_driver/src/zmcaux.cpp#L6876)
- 问题：使用了 `MOVESPIRALABS`、`MOVECLIPSEABS`、`MOVESPHERICALABS`。
- 参考：手册/SDK为 `MOVESPIRAL`、`MECLIPSEABS`、`MSPHERICAL` 系列。
- 影响：指令可能不被控制器识别。

### S4. 运动量存在重复单位换算
- 状态：待关闭
- 定位：
  - [src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L388](src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L388)
  - [src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L391](src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L391)
  - [src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L1128](src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L1128)
  - [src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L1132](src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L1132)
  - [src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L409](src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L409)
  - [src/drivers/zmc432_driver/config/zmc432_params.yaml#L18](src/drivers/zmc432_driver/config/zmc432_params.yaml#L18)
- 问题：先 `physical_to_pulses` 再调用以 UNITS 为基准的接口，语义冲突。
- 影响：位移量可能放大/缩小，存在安全风险。

### S5. 抱闸 IO 映射为临时硬编码
- 状态：待关闭
- 定位：
  - [src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L1247](src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L1247)
  - [src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L1249](src/drivers/zmc432_driver/src/zmotion_wrapper.cpp#L1249)
  - [src/drivers/zmc432_driver/src/motion_controller.cpp#L185](src/drivers/zmc432_driver/src/motion_controller.cpp#L185)
- 问题：`brake_io = 0 + axis`，且标注 `TEMP mapping`。
- 影响：可能误控 I/O，导致机械制动逻辑不安全。

## 高

### H1. `ecat_slot_id` 与轴地址映射不一致（多槽位风险）
- 状态：待关闭
- 定位：
  - [src/drivers/zmc432_driver/src/motion_node.cpp#L43](src/drivers/zmc432_driver/src/motion_node.cpp#L43)
  - [src/drivers/zmc432_driver/src/motion_node.cpp#L175](src/drivers/zmc432_driver/src/motion_node.cpp#L175)
  - [src/drivers/zmc432_driver/src/ecat_init.cpp#L409](src/drivers/zmc432_driver/src/ecat_init.cpp#L409)
  - [src/drivers/zmc432_driver/docs/txt/ZMotion_PC_Library_Programming_Guide_v2.1.2.txt#L3642](src/drivers/zmc432_driver/docs/txt/ZMotion_PC_Library_Programming_Guide_v2.1.2.txt#L3642)
- 问题：设置 `AXIS_ADDRESS` 时未将 `slot` 编入高16位。
- 影响：多槽位时轴映射可能错误。

### H2. EtherCAT 默认参数机制会覆盖用户显式配置
- 状态：待关闭
- 定位：
  - [src/drivers/zmc432_driver/include/zmc432_driver/ecat_init.hpp#L120](src/drivers/zmc432_driver/include/zmc432_driver/ecat_init.hpp#L120)
  - [src/drivers/zmc432_driver/include/zmc432_driver/ecat_init.hpp#L140](src/drivers/zmc432_driver/include/zmc432_driver/ecat_init.hpp#L140)
  - [src/drivers/zmc432_driver/include/zmc432_driver/ecat_init.hpp#L150](src/drivers/zmc432_driver/include/zmc432_driver/ecat_init.hpp#L150)
  - [src/drivers/zmc432_driver/src/ecat_init.cpp#L120](src/drivers/zmc432_driver/src/ecat_init.cpp#L120)
  - [src/drivers/zmc432_driver/src/ecat_init.cpp#L141](src/drivers/zmc432_driver/src/ecat_init.cpp#L141)
  - [src/drivers/zmc432_driver/test/test_ecat_init.cpp#L38](src/drivers/zmc432_driver/test/test_ecat_init.cpp#L38)
- 问题：`use_defaults=true` 触发 C 层重置，覆盖 `drive_pdo_mode` 等。
- 影响：用户配置看似可写，实则不生效。

### H3. 进度读写存在并发数据竞争
- 状态：待关闭
- 定位：
  - [src/drivers/zmc432_driver/src/motion_controller.cpp#L586](src/drivers/zmc432_driver/src/motion_controller.cpp#L586)
  - [src/drivers/zmc432_driver/src/motion_controller.cpp#L456](src/drivers/zmc432_driver/src/motion_controller.cpp#L456)
- 问题：执行线程写 `current_executing_cmd`，状态线程读该对象，缺少统一锁保护。
- 影响：可能出现未定义行为。

## 中

### M1. 运动进度算法不正确
- 状态：待关闭
- 定位：
  - [src/drivers/zmc432_driver/src/motion_controller.cpp#L938](src/drivers/zmc432_driver/src/motion_controller.cpp#L938)
  - [src/drivers/zmc432_driver/src/motion_controller.cpp#L955](src/drivers/zmc432_driver/src/motion_controller.cpp#L955)
  - [src/drivers/zmc432_driver/src/motion_controller.cpp#L988](src/drivers/zmc432_driver/src/motion_controller.cpp#L988)
- 问题：起始位置反复读取“当前值”，导致总距离/已完成距离失真。
- 影响：`progress` 不可信。

### M2. `wait_time` 字段未真正执行
- 状态：待关闭
- 定位：
  - [src/hypa_msgs/msg/MotionCommand.msg#L26](src/hypa_msgs/msg/MotionCommand.msg#L26)
  - [src/drivers/zmc432_driver/src/motion_topic_handler.cpp#L249](src/drivers/zmc432_driver/src/motion_topic_handler.cpp#L249)
- 问题：消息字段有定义与赋值，但执行层未使用。
- 影响：上层对节拍等待的预期落空。

### M3. 代码与仓库规范存在偏差
- 状态：待关闭
- 定位：
  - [CONTRIBUTING.md#L9](CONTRIBUTING.md#L9)
  - [CONTRIBUTING.md#L77](CONTRIBUTING.md#L77)
  - [CONTRIBUTING.md#L190](CONTRIBUTING.md#L190)
  - [src/drivers/zmc432_driver/src/ecat_init.cpp#L331](src/drivers/zmc432_driver/src/ecat_init.cpp#L331)
  - [src/drivers/zmc432_driver/src/motion_node.cpp#L180](src/drivers/zmc432_driver/src/motion_node.cpp#L180)
- 问题：存在 `i++`、行内注释、部分成员访问风格不一致。
- 影响：维护一致性下降。

## 低

### L1. CMake 重复安装同一动态库
- 状态：待关闭
- 定位：
  - [src/drivers/zmc432_driver/CMakeLists.txt#L84](src/drivers/zmc432_driver/CMakeLists.txt#L84)
  - [src/drivers/zmc432_driver/CMakeLists.txt#L121](src/drivers/zmc432_driver/CMakeLists.txt#L121)
- 问题：`libzmotion.so` 被重复 install。
- 影响：冗余，后续维护易出错。

### L2. 工作区要求的节点参数支持未完全“节点内自包含”
- 状态：待关闭
- 定位：
  - [AGENTS.md#L40](AGENTS.md#L40)
  - [AGENTS.md#L45](AGENTS.md#L45)
  - [src/drivers/zmc432_driver/src/motion_node.cpp#L27](src/drivers/zmc432_driver/src/motion_node.cpp#L27)
  - [src/drivers/zmc432_driver/launch/motion_server.launch.py#L31](src/drivers/zmc432_driver/launch/motion_server.launch.py#L31)
- 问题：`robot_name` 主要依赖 launch 层；节点直接启动场景下语义不完整。
- 影响：多机器人部署一致性依赖外部启动方式。

---

## 4. 与手册一致性核查结论（重点）

### 4.1 基本匹配项
- EtherCAT 初始化主流程顺序总体符合手册：`SLOT_SCAN -> AXIS_ADDRESS/ATYPE/DRIVE_PROFILE -> SLOT_START`。
  - 参考：[src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L19914](src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L19914)
- `WDOG` 与 `AXIS_ENABLE` 的使用方向整体正确。
  - 参考：[src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L10433](src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L10433), [src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L18508](src/drivers/zmc432_driver/docs/txt/ZMotion_Basic.txt#L18508)

### 4.2 偏差项
- 已关闭：多轴运动指令命令字和参数格式偏离官方示例。
- 已关闭：部分高级插补命令使用了文档中不存在的名称。
- 待关闭：单位模型（UNITS）在实现层被重复折算。

---

## 5. 建议修复优先级（已修复版）

### P0（先做，阻断风险）
1. 已关闭：修复 Lifecycle 状态机。
2. 已关闭：改用 SDK 现成运动封装接口替代关键自拼字符串。
3. 待关闭：统一并校准 UNITS 语义，移除重复换算。
4. 待关闭：将抱闸 IO 改为参数化映射，不允许 TEMP 默认。

### P1（高优先级）
1. 待关闭：`AXIS_ADDRESS` 映射加入 slot 高16位。
2. 待关闭：重新设计 `EcatInitInfo.use_defaults` 语义，避免覆盖用户配置。
3. 待关闭：为 `current_executing_cmd` 增加线程安全读写保护。

### P2（完善）
1. 待关闭：修复 progress 算法，保存命令起始位置快照。
2. 待关闭：实装 `wait_time` 执行行为。
3. 待关闭：清理规范偏差与 CMake 冗余 install。

---

## 6. 备注
- 本报告为静态审计与文档一致性审计结果，不代替真机联调。
- 本次“已关闭”判定基于当前代码改动与包级构建验证：
  - `colcon build --packages-select zmc432_driver --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install`
- 建议修复 P0 后立即执行最小化联调回归：
  - 连接/断开/重激活流程
  - 单轴与多轴插补
  - EtherCAT 重新扫描与重新激活
  - 位置持久化读写
