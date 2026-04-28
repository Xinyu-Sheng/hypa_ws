# zmotion_driver 安全与硬件释放检查记录

这是对 `zmotion_driver` 现有实现的静态复查记录，重点关注安全停止、硬件释放、生命周期退出和异常恢复路径。

## 检查结论

- 高风险 1: `TearDown()` 不是幂等的，且会在析构、`on_cleanup()`、`on_shutdown()`、配置失败回收中重复触发；它对 SDK 连接状态没有做保护，仍会无条件调用 `StopAll()`、`SetAxisEnable(false)`、`ShutdownEthercat()`、`Disconnect()`。
- 高风险 2: `StopAll()`、`SetAxisEnable(false)`、`ShutdownEthercat()` 的失败大多被吞掉，只记录日志，生命周期层仍可能返回成功，导致“软件认为已急停/已释放，但硬件实际未释放”。
- 中风险: `enable_axis_on_activate=true` 时，`ActivateNode()` 会直接给所有轴发使能命令；默认配置里 `drive_enable=0` 且 launch 只请求 `configure`，但一旦手动激活，仍存在上电前硬件状态未核验的问题。

## 现有检查

1. 退出链路存在重复释放风险。`Impl::~Impl()` 会自动调用 `TearDown()`，而 `on_cleanup()`、`on_shutdown()`、`on_configure()` 失败分支也会再次调用它，因此释放逻辑必须是幂等的。
2. `TearDown()` 内部先执行停止和去使能，再执行 `ShutdownEthercat()` 和 `Disconnect()`。一旦前序路径已经断开，后续的 SDK 运动命令仍会被调用，边界不安全。
3. `TriggerEmergencyStopLocked()` 会在急停时调用 `StopAll()` 和逐轴 `SetAxisEnable(false)`，但不检查返回值。也就是说，急停状态可以被置位，而硬件是否真的停下并没有被确认。
4. `DeactivateNode()` 同样忽略 `StopAll()` 和 `SetAxisEnable(false)` 的返回值，生命周期“deactivate 成功”并不能证明硬件已经真正释放。
5. `ShutdownEthercat()` 对 `DRIVE_CONTROLWORD`、`WDOG=0`、`SLOT_STOP()` 的失败只给出 WARN，最后仍返回 Success，不能把关断失败暴露给上层。
6. `main.cpp` 里只是 `executor.spin()` 结束后直接 `rclcpp::shutdown()`，没有显式触发 lifecycle `shutdown` 过渡。真正的硬件释放主要依赖节点回调和析构兜底。
7. 当前测试目录里没有看到针对 `TearDown()`、`ShutdownEthercat()`、急停失败回传的专门回归测试，覆盖不足。

## 补充发现（硬件资源管理遗漏）

1. SDK 全局初始化缺少反初始化。`ZMotionSdkWrapper::Connect()` 里只做了一次 `ZMC_LinuxLibInit()`，并且用静态布尔值保证只初始化一次，但在 `third_party/zmotion_sdk/zmotion.h` 里只找到 `ZMC_LinuxLibInit()`，没有看到对应的 `LinuxLibUnInit` 之类释放接口。对进程内反复创建和销毁节点、或者测试里重复加载驱动的场景，这属于全局资源释放的遗漏点。
2. `CloseLogFile()` 的资源管理风格和另外两路 CSV 文件不一致。`WriteLogLocked()` 虽然通常在 `this->mutex` 保护下执行，但 `CloseLogFile()` 本身没有和 `command/joint_state` 一样的独立互斥锁；`TearDown()` 也是先 `ResetInterfaces()` 再关闭主日志流。当前实现依赖大多数回调在 `configured=false` 后直接返回，所以这还不是最强的现成故障点，但属于容易在后续新增日志路径时演变成关闭/写入脆弱点。
3. `Disconnect()` 会把关闭失败隐藏成“已断开”状态。它先调用 `ZAux_Close()`，随后无论返回码如何都会把 `connected` 设为 `false`、`handle` 置空。上层在 `TearDown()`、`on_configure()` 失败回收里又普遍忽略 `Disconnect()` 的返回值，所以一旦底层 close 失败，内部状态会提前清空，后续既无法重试，也无法区分“真断开”和“假断开”。
4. `Connect()` 会把 `ZMC_LinuxLibInit()` 和 `ZAux_SetTimeOut()` 的返回值都丢掉，而且在 `ZMC_LinuxLibInit()` 之后立刻把 `linux_lib_inited` 置为 `true`。这意味着即使 SDK 初始化或超时设置失败，后续连接流程仍会继续，并且后续重试也不会再次走初始化路径，失败会被伪装成“已准备好”。
5. 配置阶段的清停和清故障同样是 best-effort。`SlotScan()` 里的 `SLOT_STOP()` 被直接忽略，`InitEthercat()` 里的 `ZAux_Direct_Rapidstop()`、`DRIVE_CONTROLWORD()` 序列和 `ZAux_BusCmd_DriveClear()` 也都不检查返回值，然后才继续 `StartSlotOp()` 和后续配置。这样一来，configure 可能在总线还没真正停稳、或者驱动仍带故障的情况下继续推进，最后把“配置成功”暴露给上层。

## 专门检查：命令队列、IO 回调和生命周期切换

1. 这里其实没有真正的“命令队列”。`OnVelocityCommand()`、`OnMimicCommand()` 都是同步构造 `PendingCommand`，随后直接进入 `DispatchCommand()` / `DispatchCommandLocked()`；命令不会在内存里排队等待后台线程消费，所以不存在典型的队列泄漏或队列积压释放问题。真正的边界是：一旦 `configured`、`active` 或 `emergency_stop` 变更，后续命令会被直接丢弃，而不是被缓冲起来。
2. `DeactivateNode()` 只做了停轴、去使能和 publisher 反激活，没有关闭 `feedback_timer` / `io_timer`，这些定时器只会在 `TearDown()` 的 `ResetInterfaces()` 里被 reset。结果是节点进入 inactive 之后，IO 轮询仍然继续执行。
3. `PollIoInputs*()` 只检查 `configured`，不检查 `active`。因此在 inactive 但尚未 cleanup 的阶段，IO 轮询仍然会继续读硬件；更关键的是，`DispatchIoTriggerNew2Locked()` / `DispatchIoTriggerNewLocked()` / `DispatchIoTriggerLocked()` 在 `emergency_stop_on_high` 分支里没有先做 `configured && active` 门禁，只要输入高电平到来，就可能再次调用 `TriggerEmergencyStopLocked()`。
4. `OnBrakeCommand()` 也没有 `configured` 或 `active` 的门禁。`cmd/brake=true` 在 deactivate 到 cleanup 之间的窗口仍然可以触发 `TriggerEmergencyStopLocked()`，从而对已经准备释放、甚至已经断开的 handle 再次发 `StopAll()` 和 `SetAxisEnable(false)`。
5. `DeactivateNode()` 在调用 `StopAll()` / `SetAxisEnable(false)` 之前就把 `emergency_stop=false` 复位了，而且不检查这些调用的结果。若硬件 stop 失败，软件层会过早丢失安全锁，后续生命周期重新 activate 时会把“未确认安全”的硬件当成正常状态恢复。

## 专门检查：控制逻辑安全

1. 没有看到典型的互斥环死锁，但控制、反馈、IO、生命周期和急停都共用同一把 `mutex`，而且 `PublishFeedback()`、`PollIoInputs*()`、`DispatchCommandLocked()`、`DeactivateNode()`、`ActivateNode()`、`TearDown()` 都在锁内直接做硬件访问。结果是：任何一个较慢的 SDK 调用都会阻塞所有其他控制路径，急停和失活不是抢占式的，而是排队等待前一个硬件调用结束。
2. `PublishFeedback()` 里的位置、速度、力矩和轴状态读取失败，只会记录 `feedback_err`，并回退到 last-known 或 NaN，不会自动触发急停。也就是说，反馈链路失效时系统仍可能继续发命令，控制逻辑没有把“失去反馈”当成失控条件处理。
3. 多轴绝对运动默认启用了 `ZMOTION_USE_MOVEMODIFY_FOR_MULTIAXIS`。注释明确写着这个模式需要理解 `MoveModify` 的时序语义，但当前构建里它是打开的；一旦某个轴的 `MoveModify` 失败，代码会直接对整组轴回退到 `MoveAbs`，中间没有回滚已修改轴的状态。这是一个容易被忽略的控制一致性隐患。
4. `ActivateNode()` 的失败分支只做 best-effort 的 `StopAll()` 和逐轴 `SetAxisEnable(false)`，但没有把 `emergency_stop` 置位，也没有强制清理或断开。若硬件在激活失败时已经进入半使能状态，而 stop/disable 又失败，上层会看到“激活失败”，但不会得到一个强制锁定的安全态。
5. `moving_axes_direction` 只在 `StopAll()`、`CancelAxis()` 或 `Disconnect()` 成功时清空，失败时会保留旧方向。这样一来，如果停机失败后又进入新的生命周期，后续同方向速度命令可能因为缓存认为“已经在运动”而被静默跳过，导致软件状态和真实运动状态脱节。
6. `DeactivateNode()` 把 `emergency_stop=false` 复位得比硬件停机更早；如果随后 `StopAll()` / `SetAxisEnable(false)` 失败，软件就会提前释放安全锁。配合上面那条缓存状态问题，这会让后续控制逻辑更容易在“看起来已恢复、实际上未确认安全”的前提下重新下发命令。
7. 多轴速度命令不是原子下发。`ExecuteCommandLocked()` 在 `kVelocity` 分支里是逐轴调用 `CommandVelocity()`，如果前面某个轴已经开始运动，而后面的轴返回失败，函数只会直接返回错误，不会对前面已成功的轴做 `CancelAxis()` / `StopAll()` 回滚。这会把“一个多轴命令失败”变成“部分轴已经动起来”的真实失控窗口。

## 专门检查：ROS 2 配置与生命周期

1. 启动文件只发了 `configure`，没有自动 `activate`，也没有生命周期管理器去编排后续状态。结果是这个节点默认启动后处于 `inactive`，控制命令看起来已经在线，但实际上不会真正执行；如果运维人员只按普通 ROS 2 节点理解，很容易误判系统“已经启动完成”。
2. `emit_configure` 不是挂在 `OnProcessStart` 之类的启动事件后面，而是随 LaunchDescription 直接发出。对于慢启动机器或者资源紧张环境，生命周期 transition 请求可能早于节点生命周期服务就绪，形成启动竞态。这个点通常不会每次都复现，但一旦发生会表现为“偶发 configure 失败/状态未切到 inactive”。
3. `main.cpp` 在 `executor.spin()` 结束后立刻调用 `rclcpp::shutdown()`，而真正的 `on_shutdown()` 并不在默认退出路径里被显式触发。也就是说，正常退出时硬件释放主要依赖 `Impl::~Impl()` 的析构兜底，而不是生命周期状态机本身。这让退出行为更依赖对象销毁顺序，也让 `TearDown()` 里的 ROS 访问更接近“上下文已关闭后再做清理”的边界。
4. `LoadParameters()` 对每个必需参数都直接 `get_parameter(...).as_*()`，没有把缺参、类型错配、命名空间不匹配转换成显式的 lifecycle `FAILURE`。如果参数 YAML 里有拼写错误、节点名/namespace 不匹配，或者 launch 传入的参数文件根本没被正确加载，当前代码更容易抛异常并把整个进程打掉，而不是以受控方式停在 `unconfigured`。
5. 启动命令和文档默认值存在 namespace 现实差异：launch 默认 namespace 是 `hypa`，而生命周期说明文档里的示例命令用的是 `/zmotion_driver`。虽然文档里写了“带 namespace 时要改前缀”，但实际运维时如果沿用示例命令，很容易对着错误的生命周期路径操作，误以为节点卡住或没起来。
6. `use_sim_time` 通过 launch 注入是正确的，但当前代码没有围绕时间源缺失做额外保护。若仿真场景下忘记带 `use_sim_time=true`，日志和反馈时间戳会直接落到系统时间；若反过来启用仿真时间但 `/clock` 不稳定，反馈/IO 的时间基准也会抖动。这不是代码崩溃点，但会直接影响 ROS 2 侧的诊断和回放一致性。
7. 关键数值参数只做了类型读取，没有做范围校验。`controller.feedback_period_ms`、`controller.io_period_ms`、`controller.min_remain_buffer`、`ecat.init.drive_axis_start`、`ecat.init.drive_axis_num`、`ecat.init.slot_id` 都直接进入定时器创建或硬件命令，0/负值/越界值不会在配置阶段被拦下，可能导致 busy-loop 定时器、无效轴号命令，或者让安全阈值失去意义。
8. `ecat.init.drive_enable` 是一条配置阶段的真实上电通路。`InitEthercat()` 在 `cfg.DriveEnable == 1` 时会直接调用 `SetAxisEnable(..., 1)`，这意味着只要参数改成 1，轴可能在 configure 阶段就被使能，而不是等到 lifecycle 的 activate。这个路径当前没有在文档里被单独说明，属于容易被忽略的生命周期绕过点。
9. `CreateInterfaces()` 没有异常保护。`create_publisher()`、`create_subscription()` 和 `create_wall_timer()` 直接用参数里的 topic 字符串创建 ROS 接口，整个函数和 `on_configure()` 都没有 try-catch 包裹。如果 topic 名称非法，或者创建接口时触发 rclcpp 异常，这次 configure 可能直接抛出而不是优雅返回 FAILURE；这和前面的“参数读取失败”是两条不同的故障路径。

## 本轮新增：配置映射与时间语义

1. `io.input_ids` 没有做范围或重复性验证。代码会把每个 IO ID 直接用于 `GetInput()`，还会在 `io.state_topics` 为空时自动拼接成 `io/input_<id>`。如果配置里出现负数、越界或重复 ID，可能带来无效硬件访问、重复状态发布，甚至生成非法 topic 名称。
2. EtherCAT 映射数组没有按实际轴数/节点数做完整校验。`ecat.init.drive_pdo_mode`、`ecat.init.node_io_id`、`ecat.init.node_aio_id`、`ecat.init.dc_offset_flag`、`ecat.init.dc_offset_time` 只是按输入长度拷贝到 128 项数组里，长度不足的尾部会保留 0 或默认值。`InitEthercat()` 又会直接读取这些数组去写 `DRIVE_PROFILE`、`NODE_IO`、`NODE_AIO` 和 DC 偏移，所以一份截断的 YAML 可以静默改变控制器映射，而不会在配置阶段被拦下。
3. IO debounce 的状态没有在生命周期恢复时清理。`last_io_change_ms` 只在 `LoadParameters()` 时初始化，`DeactivateNode()` 和 `TearDown()` 都没有清空它；而 `PollIoInputs*()` 的去抖逻辑又使用 ROS 时间做差值判断。如果启用了 debounce，或者 `/clock` 在仿真中跳变，重新激活后的一段时间内，合法的 IO 边沿可能被错误抑制。
4. `io.trigger_modes` 和 `io.emergency_stop_on_high` 都没有长度校验，也没有对非法枚举值报错。`io.trigger_modes` 列表比 `io.input_ids` 短时，尾部 IO 会静默落到 `kNone`；`io.emergency_stop_on_high` 列表比 `io.input_ids` 短时，尾部 IO 会被静默补成 `false`；而 `io.trigger_modes` 的值不在 0 到 5 之间时也会走 `default` 分支变成 `kNone`。这会把原本应触发急停、联动或边沿动作的输入，悄悄退化成无动作。

## 本轮新增：输入与日志安全

1. 日志和 CSV 文件没有大小限制或轮转机制。`controller.log_file`、`feedback.record_joints_csv_file`、`feedback.record_commands_csv_file` 都是按运行时追加写入，且文件名只做了时间戳和唯一化处理，没有单文件大小上限。对长时间运行或频繁重启的系统，这会带来真实的磁盘耗尽风险。
2. 写文件失败不会抛异常，但也没有被显式检查。`WriteLogLocked()`、`WriteJointStateCsv()`、`WriteCommandCsv()` 都只是执行流式写入和必要的 `flush()`，没有检查 `bad()/fail()`，也没有设置 `exceptions()`。因此磁盘满、权限变化或文件系统错误更可能表现为“静默丢日志/丢 CSV”，而不是程序崩溃。
3. 速度/位置类输入缺少 `NaN/Inf` 校验。`OnVelocityCommand()` 和 `OnMimicCommand()` 只检查数组长度，没有对数值做 `std::isfinite()` 过滤；这些值会继续进入 `DispatchCommand()`、`ExecuteCommandLocked()` 和 SDK 调用链。对运动控制来说，非有限数值是实际风险输入，不应直接下发。
4. IO 轮询实现保留了三套并行路径：`PollIoInputsLegacy`、`PollIoInputsNew`、`PollIoInputsNew2`。更重要的是，`src` 里把 `ZMOTION_DRIVER_USE_NEW_POLL_IO_INPUTS` 和 `ZMOTION_DRIVER_USE_NEW_POLL_IO_INPUTS_2` 直接定义死了，源码注释里说的“通过取消注释或 build flags 切换”实际上并不成立。也就是说，这个 IO 路径选择不是一个真正可配置的开关，而是被当前文件硬编码住的。
5. `ZMC_LinuxLibInit()` 依赖函数内静态布尔值 `linux_lib_inited` 做一次性初始化保护，这个标志本身不是并发安全的。当前调用路径在节点互斥锁下基本是串行的，所以风险偏低，但如果未来把 Connect 迁到并发路径，或者新增并行实例，这里会变成真实竞态点。

## 相关文件

- [src/zmotion_driver_node.cpp](../../src/zmotion_driver_node.cpp) - 生命周期回调、`TearDown()`、`TriggerEmergencyStopLocked()`、`ActivateNode()`、`DeactivateNode()`
- [src/zmotion_sdk_wrapper.cpp](../../src/zmotion_sdk_wrapper.cpp) - `Disconnect()`、`StopAll()`、`SetAxisEnable()`、`ShutdownEthercat()`
- [src/main.cpp](../../src/main.cpp) - 进程退出路径
- [launch/zmotion_driver.launch.py](../../launch/zmotion_driver.launch.py) - 启动时生命周期切换
- [config/zmotion_driver.yaml](../../config/zmotion_driver.yaml) - 激活和 EtherCAT 默认安全参数

## 建议后续验证

1. 构造一次连接失败，再验证 cleanup 路径不会继续向已断开的句柄发送运动/释放命令。
2. 连续调用 cleanup 或 shutdown 两次，确认第二次是无副作用的幂等释放。
3. 人为注入 `StopAll()` 或 `SetAxisEnable(false)` 失败，确认节点不会继续报告“安全停止成功”。
4. 修复后做一次对应范围的构建验证，至少检查 `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install`。
