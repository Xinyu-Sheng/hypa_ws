# zmotion_driver 生命周期切换

这个驱动是 ROS 2 Lifecycle 节点。建议先检查当前状态，再做切换，避免在错误状态下直接操作硬件。

## 1. 先确认节点状态

先启动驱动，然后查看状态：

```bash
ros2 lifecycle get /zmotion_driver
```

如果节点带了命名空间，把路径改成：

```bash
ros2 lifecycle get /<namespace>/zmotion_driver
```

## 2. 常用切换顺序

从未配置状态进入运行状态，通常按下面顺序：

```bash
ros2 lifecycle set /zmotion_driver configure
ros2 lifecycle set /zmotion_driver activate
```

停止运行时，按下面顺序：

```bash
ros2 lifecycle set /zmotion_driver deactivate
ros2 lifecycle set /zmotion_driver cleanup
```

## 3. 状态说明

- configure：连接控制器，初始化 EtherCAT，创建话题和定时器。
- activate：使能轴，开始对外发布反馈并接收控制命令。
- deactivate：停止任务，关闭轴使能。
- cleanup：释放接口并断开控制器。

## 4. 故障处理

- 如果硬件 remain buffer 低于阈值或读取失败，驱动会立即进入 emergency_stop，并关闭所有轴使能。
- 进入 emergency_stop 后，不会自动恢复；`brake` 释放也不会自动重新使能轴。
- 恢复方式是走生命周期流程，至少执行 `deactivate`，再按需要 `cleanup` -> `configure` -> `activate`。

## 5. 注意事项

- activate 不是纯软件开关，会实际使能轴。
- deactivate 会执行 StopAll，并关闭轴使能。
- 如果节点带 namespace，命令里的节点名也要加上对应前缀。
- 这个包没有默认的生命周期管理器脚本，所以最简单的方式就是直接用 ros2 lifecycle 命令。
