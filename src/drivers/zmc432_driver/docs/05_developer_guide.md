# 开发者指南

本节为欲阅读或修改源码的开发者准备，涵盖代码结构、主要类与扩展点。

## 目录结构

```
zmc432_driver/
├── include/zmc432_driver/        # 头文件
│   ├── ecat_init.hpp            # EtherCAT 初始化接口（现在包含总线轴/IO映射、PDO配置和启动逻辑）
│   ├── motion_controller.hpp    # 控制器类
│   ├── motion_topic_node.hpp    # ROS 话题节点
│   ├── zmotion_wrapper.hpp      # 低级 SDK 封装
├── src/                         # 实现
│   ├── ecat_init.cpp
│   ├── motion_controller.cpp
│   ├── motion_node.cpp          # main + 参数
│   ├── motion_topic_node.cpp
│   ├── zmotion_wrapper.cpp
│   └── zmcaux.cpp               # SDK 源文件
├── launch/motion_server.launch.py
├── test/ (包含 test_enable.cpp/test_ecat_init.cpp)
```

## 关键类

### `ZMotionWrapper`
封装 ZMotion SDK，提供面向对象的接口，如 `connect()`, `queue_motion()`, `set_axis_enable()`。
实现见 `zmotion_wrapper.cpp`。

### `MotionController`
业务逻辑层，负责参数配置、运动队列、状态聚合。调用 `ZMotionWrapper` 实际下发指令。
新增使能接口：`enable_axis()`、`get_axis_enable()`，并在 `queue_motion()` 中添加使能检查。

### `MotionTopicNode`
ROS 话题交互层。负责声明参数、订阅 `MotionCommand` 并调用 `MotionController`，发布 `MotionStatus`。

> **拓展提示**: 当前轴使能接口只在控制器 API 中存在(`enable_axis`)并由测试程序 `test_enable` 调用。若希望通过 ROS 服务或额外的话题暴露此功能，可在 `MotionTopicNode` 中添加对应的服务端，并将请求转发到 `MotionController`。

## 添加新功能

- 添加新参数：在 `motion_node.cpp` 的 `declare_parameter()` 添加并初始化。
- 扩展消息：修改 `hypa_msgs` 中的 `.msg` 文件并重新编译。
- 增加测试：在 `src/test/` 添加 CPP 并在 `CMakeLists.txt` 加入可执行目标。

## 样例代码片段

```cpp
// 检查使能并移动
auto status = controller->get_axis_enable(axis);
if (status && *status) {
  controller->queue_motion(cmd);
} else {
  RCLCPP_WARN(this->get_logger(), "axis %d not enabled", axis);
}
```
