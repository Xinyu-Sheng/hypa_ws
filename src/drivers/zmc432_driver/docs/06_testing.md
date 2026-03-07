# 测试与验证

本节介绍如何构建测试程序、运行例程以及执行手动验证。

## 编译测试

```bash
cd /home/xinyu/Projects/HKU/hypa_ws
colcon build --packages-select zmc432_driver --cmake-args -DBUILD_TESTING=ON
```

## 可用测试程序

- `test_enable`：验证轴使能/失能逻辑。
- `test_ecat_init`：连接控制器并执行 EtherCAT 初始化。

```bash
source install/setup.bash
ros2 run zmc432_driver test_enable
ros2 run zmc432_driver test_ecat_init 192.168.0.11
```

> 注意：这两个可执行文件会链接 `libzmotion.so`，请确保其在 `LD_LIBRARY_PATH` 或 `install/zmc432_driver/lib` 中。

## 手动检查

- 使用 `ros2 topic echo` 观察 `/motion_status`，确认 `axis_enabled` 和 `axis_statuses` 变化。
- 执行简单运动，查看 `progress` 字段是否达到 100%。
- 在 EtherCAT 环境下尝试启用 `perform_ecat_init` 参数，检查日志是否显示扫描成功。
- 遇到故障时查看 watchdog 日志，提取错误码并参考 `docs/04_advanced.md` 中的错误处理章节。
