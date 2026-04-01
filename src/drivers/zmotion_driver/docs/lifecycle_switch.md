# zmotion_driver 生命周期切换

这个驱动是 ROS 2 Lifecycle 节点。建议先检查当前状态，再做切换，避免在错误状态下直接操作硬件。

当前版本的运动语义需要特别区分：速度控制是连续运行，位置控制是离散点位运动。位置命令到达每个目标点时会自然减速并停住，不会自动做连续过点；如果需要连续插补，需要使用 ZMotion 的 SP / MERGE 机制，但当前包默认没有启用这条路径。

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

## 4. 注意事项

- activate 不是纯软件开关，会实际使能轴。
- deactivate 会执行 StopAll，并关闭轴使能。
- 如果节点带 namespace，命令里的节点名也要加上对应前缀。
- 这个包没有默认的生命周期管理器脚本，所以最简单的方式就是直接用 ros2 lifecycle 命令。
- 如果你看到位置命令“段与段之间会停一下”，这是当前实现的正常行为，不是 speed 被设成 0；位置模式默认就是离散点位控制。
- 如果后续要做连续过点，必须显式引入 SP / MERGE 相关配置和对应的 SDK 封装，不能只靠连续发布位置消息。