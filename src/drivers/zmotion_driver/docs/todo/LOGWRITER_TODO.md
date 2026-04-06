LOGWriter 实现任务清单（简洁、可执行）

目标
- 实现一个后台 `LogWriter` 组件，用单独线程按批写入日志/CSV，减少主线程 I/O 阻塞并支持轮转、刷新与优雅关闭。
- 将 `WriteLogLocked`、`WriteJointStateCsv`、`WriteCommandCsv` 的写入改为推入 `LogWriter`（最小侵入）。

整体设计（高层）
1. 单一后台线程：`LogWriter` 管理一个后台线程负责落盘。不要在主路径做磁盘 I/O。 
2. 线程安全队列：主线程调用 `push(line)` 将已格式化的字符串入队；后台线程批量 draining 并写入文件。
3. 批写策略：支持 `batch_size`（行数阈值）和 `flush_interval_ms`（时延阈值），到达任一阈值则写入/flush。
4. 多文件支持：同一个 `LogWriter` 可管理多条输出流（event log / joint_state / commands），或为每种文件单独实例（实现上更简单也更灵活）。
5. 轮转与保留：按文件大小或时间轮转，新文件名带时间戳，保留 N 个文件。
6. 优雅关闭：提供 `stop_and_flush()` 阻塞直到队列清空并线程退出，在 `TearDown()` 中调用，但不要在持有主 mutex 的上下文中调用。

API（建议）
- class LogWriter {
-  public:
-    LogWriter(const std::string &path, size_t batch_size = 64, size_t flush_interval_ms = 50, size_t max_file_size = 0 /*MB=0禁用*/, size_t retention_files = 5);
-    ~LogWriter();
-    void push(const std::string &line); // 非阻塞，如果队列满可丢弃或返回错误（策略可配置）
-    void push_and_flush(const std::string &line); // 保证写入并触发一次 flush（用于关键事件）
-    void flush_now();
-    void stop_and_flush(); // 阻塞直到队列写入并线程退出
- };

内部实现要点
- 队列：`std::deque<std::string>` + `std::mutex` + `std::condition_variable`。
- 后台线程循环：
  - 等待 `cond_var` 或超时 `flush_interval_ms`。
  - 取出最多 `batch_size` 或所有行，合并为单个写入操作写入流（一次 `operator<<` 多行或 `std::string` 合并并 `write()`）。
  - 当文件大小超过 `max_file_size`（MB）时轮转：关闭当前文件，重命名/移动并打开新文件，基于时间戳命名。
  - 定期 `flush()`：当批次写入或到达时间间隔时 `flush()`；对 `push_and_flush` 立即 `flush()`。
- 失败处理：写入失败（IO error）记录到 stderr 或内部备用流，并重新尝试若干次，再告警。

集成点（在 `zmotion_driver_node.cpp`）
1. 在 `Impl` 成员中添加 `std::unique_ptr<LogWriter> log_writer_events;`、`log_writer_joint_state;`、`log_writer_commands;`（或单一 `LogWriter` 管理多文件）。
2. 在 `LoadParameters()` 新增配置项：`controller.log_batch_size`、`controller.log_flush_interval_ms`、`controller.log_max_size_mb`、`controller.log_retention_files`、和 `feedback.record_commands_csv_enabled` 相关配置。
3. 在 `on_configure()`：创建并启动 `LogWriter` 实例（调用构造函数即可启动线程），并替换 `OpenLogFile`/`OpenJointStateFile` 的调用为 `LogWriter` 管理文件打开（或仍保留 open 但改为 `LogWriter::push(...)`）。
4. 修改写入点：
   - `WriteLogLocked` 改为 `log_writer_events->push(line)`（不再直接写文件）；
   - `WriteJointStateCsv` 改为 `log_writer_joint_state->push(line)`；
   - `WriteCommandCsv` 改为 `log_writer_commands->push(line)`。
5. 在 `TearDown()`：调用 `log_writer_*.stop_and_flush()`（确保在不持关键 mutex 时调用，若必须在持锁路径，先释放锁再调用）。

性能/安全建议
- 默认 `batch_size=64`、`flush_interval_ms=50`。对 10ms 控制频率，使用这些值将把写 syscall 频率大幅降低。
- 为关键事件（`control_tx` / `control_err`）使用 `push_and_flush`，确保关键记录能落盘。
- 队列策略：可配置为有界队列（例如上限 100k 行），超出时选择丢弃低优先级行或阻塞短时（慎用阻塞）。
- 日志格式化：建议主线程完成字段序列化（生成 CSV 行），后台只做写操作，减少字符串分配/拼接在后台；若格式化耗时显著，可后移到后台（权衡 CPU 与主线程负载）。

测试与验证
1. 单元：实现 `LogWriter` 后写单元测试（在 tests/ 下）覆盖批写、轮转、stop_and_flush、异常处理。
2. 集成：构建包并运行节点，使用 100Hz 模拟输入，分别测量：
   - `write()` syscall 频率（`strace -f -c -e trace=write -p <pid>`）
   - 主循环最大/平均延迟（可在代码中临时插桩或用外部探针）
   - 日志完整性（在正常 shutdown 与强制 kill 场景下检查丢失行数）
3. 回归：确认 `commands.csv` 可与 `joint_state` 用 `stamp_ns` 对齐，关键事件 `tx/err` 不丢失。

文件与代码变更清单（建议）
- 新增：`src/drivers/zmotion_driver/include/zmotion_driver/log_writer.hpp`
- 新增：`src/drivers/zmotion_driver/src/log_writer.cpp`
- 修改：`src/drivers/zmotion_driver/src/zmotion_driver_node.cpp`
  - 引入并使用 `LogWriter` 实例
  - 修改生命周期钩子 `on_configure`、`TearDown` 等
- 可选：增加 `tests/test_log_writer.cpp`（gtest）用于 CI

实施时间估算（粗略）
- 最小可用版本（单文件、批写、flush、stop）: 2–4 小时。
- 完整版本（多文件、轮转、保留、错误重试、单元测试）: 1–2 天。

备注
- 初版先实现可靠、易懂的同步实现（mutex + condvar）。后续再换成无锁队列或优化字符串处理。
- 注意在 `TearDown()` 调用 `stop_and_flush()` 时不要在持有关键资源锁的上下文中阻塞。


---

如果你确认，我可以立即开始实现 `LogWriter`（先做最小可用版本）。