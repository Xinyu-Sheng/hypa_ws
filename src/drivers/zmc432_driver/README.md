# zmc432_driver

HYPA 硬件驱动包，为 ZMC432 运动控制器提供 ROS 2 话题接口。

> 📖 本项目的**完整文档**按难度分层保存在 `docs/` 目录，推荐从 [docs/index.md](docs/index.md) 开始。初学者请依序阅读“快速上手”、“接口说明”等章节。

## 快速示例

构建并启动最基本的节点：

```bash
cd /home/xinyu/Projects/HKU/hypa_ws
colcon build --packages-select hypa_msgs zmc432_driver
source install/setup.bash
ros2 launch zmc432_driver motion_server.launch.py
```

发送一个简单运动命令并观察状态：

```bash
ros2 topic pub /hypa/motion_command hypa_msgs/msg/MotionCommand \
  '{axes: [0], positions: [100.0], velocities: [10.0], motion_type: 1}'
ros2 topic echo /hypa/motion_status
```

读者可在 `docs/01_quick_start.md` 获取更多示例及故障排除技巧。

## 导航

- 🧭 **快速上手**：`docs/01_quick_start.md`
- 🔌 **接口 & 参数**：`docs/02_interface.md`、`03_configuration.md`
- 🛠️ **高级 & 开发**：`docs/04_advanced.md`、`05_developer_guide.md`
- 🧪 **测试说明**：`docs/06_testing.md`

## 许可证 & 维护

Apache‑2.0 许可。
维护者: Xinyu Sheng <sheng.xin.yu@faxmail.com>

---
*如果你是第一次访问此仓库，请一定参阅 `docs/index.md` 获取完整说明。*
