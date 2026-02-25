# AI Agent Instructions for This Workspace

## 🛠️ Build System Rules (ROS 2 + colcon)

### Clangd 支持（代码补全、跳转、诊断）

所有构建命令**必须**包含以下 CMake 参数以生成 `compile_commands.json`：

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

---

## 📦 CMake 依赖管理

### ROS 2 包的依赖处理

- **ROS 依赖**（通过 `find_package()` 找到的包）：使用 `ament_target_dependencies(target pkg1 pkg2 ...)`
  - 自动处理 include、link、compile definitions
  - 适用于 `rclcpp`、`hypa_msgs` 等 ROS 包
  
- **非 ROS 依赖**（第三方库）：使用 `target_link_libraries()`
  - 如 ZMotion SDK 的 `libzmotion.so`

- **项目头文件**：使用 `target_include_directories()`
  - 项目自己的 include 路径
