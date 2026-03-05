# 电机使能功能实现文档

## 概述

本文档描述了 HYPA 工作区中 ZMC432 运动控制器的电机使能功能实现。

## 实现的功能

### 1. 底层 API 封装 (ZMotionWrapper)

**新增方法：**
- `set_axis_enable(int axis, bool enable)` - 设置轴使能状态
- `get_axis_enable(int axis) -> optional<bool>` - 读取轴使能状态

**实现细节：**
- 使用 ZMotion SDK 的 `ZAux_Direct_SetAxisEnable()` 和 `ZAux_Direct_GetAxisEnable()` 函数
- 包含完整的错误处理和参数验证
- 支持轴号范围检查 (0-31)

### 2. 控制器层接口 (MotionController)

**新增方法：**
- `enable_axis(int axis, bool enable)` - 使能/失能单个轴
- `enable_all_axes(bool enable)` - 使能/失能所有已配置轴
- `get_axis_enable(int axis) -> optional<bool>` - 读取单个轴使能状态

**功能特性：**
- 批量操作支持
- 错误聚合和报告
- 连接状态检查

### 3. 数据结构扩展

**AxisStatus 结构体新增字段：**
```cpp
struct AxisStatus {
  // ... 原有字段
  bool enabled = false;  // 是否使能
};
```

**MotionStatus 消息新增字段：**
```
bool[] axis_enabled  # 各轴的使能状态
```

### 4. 运动控制逻辑增强

**安全检查：**
- 在 `queue_motion()` 中添加使能状态检查
- 未使能的轴无法执行运动命令
- 提高系统安全性

### 5. ROS2 接口更新

**状态发布：**
- MotionTopicNode 自动发布使能状态
- 实时监控各轴使能状态

## 使用示例

### C++ API 使用

```cpp
auto controller = std::make_unique<zmc432_driver::MotionController>();

// 初始化控制器
auto init_result = controller->initialize("192.168.0.11");
if (init_result) {
    // 处理连接错误
    return;
}

// 配置轴
controller->configure_axis(0, 1.0, 10.0, 100.0, 100.0);

// 使能轴
auto enable_result = controller->enable_axis(0, true);
if (enable_result) {
    // 处理使能错误
    return;
}

// 检查使能状态
auto enable_status = controller->get_axis_enable(0);
if (enable_status && enable_status.value()) {
    // 轴已使能，可以执行运动
    controller->queue_motion(cmd);
}
```

### ROS2 Topic 监控

```bash
# 监控运动状态（包含使能状态）
ros2 topic echo /hypa/motion_status
```

## 测试

### 编译测试程序

```bash
cd /home/xinyu/Projects/HKU/hypa_ws
colcon build --packages-select zmc432_driver --cmake-args -DBUILD_TESTING=ON
```

### 运行测试

```bash
source install/setup.bash
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./install/zmc432_driver/lib/zmc432_driver \
./install/zmc432_driver/lib/zmc432_driver/test_enable
```

## 文件修改清单

### 新增文件
- `src/test_enable.cpp` - 使能功能测试程序

### 修改文件
- `include/zmc432_driver/zmotion_wrapper.hpp` - 添加使能方法声明
- `src/zmotion_wrapper.cpp` - 实现使能方法
- `include/zmc432_driver/motion_controller.hpp` - 添加使能接口
- `src/motion_controller.cpp` - 实现使能功能和状态检查
- `src/motion_topic_node.cpp` - 发布使能状态
- `hypa_msgs/msg/MotionStatus.msg` - 添加使能状态字段
- `CMakeLists.txt` - 添加测试程序支持

## 注意事项

1. **安全性**：运动前会自动检查轴使能状态
2. **错误处理**：所有使能操作都包含完整的错误处理
3. **兼容性**：保持与现有代码的完全兼容
4. **性能**：使能状态读取是轻量级操作

## 未来改进

- [ ] 添加使能状态变化的回调机制
- [ ] 支持使能超时设置
- [ ] 添加使能状态的历史记录
- [ ] 实现使能状态的持久化存储

## 总结

电机使能功能已完全集成到 HYPA 运动控制系统中，提供了从底层 API 到高层 ROS2 接口的完整支持。该实现确保了系统安全性和易用性，同时保持了良好的代码结构和错误处理机制。
