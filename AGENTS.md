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

---

## 📝 Commit 规范

**注意**：所有 commit 操作均基于当前**暂存区（staged）**的内容。内容要求简洁，一律使用双引号 + 自然换行方式，不要用任何 escape 换行符或特殊 quoting。

### Commit Message 格式

```
<类型>: <简要描述>

<详细说明>

<影响包列表>
```

#### 类型（Type）
- `feat`: 新功能
- `fix`: 修复 bug
- `docs`: 文档变更
- `style`: 代码格式调整（不影响功能）
- `refactor`: 代码重构
- `test`: 测试相关
- `chore`: 构建/工具变更

#### 影响包列表（Affected Packages）
在 commit message 末尾列出受影响的 ROS 2 包，格式：
```
affected-packages: hypa_msgs, hypa_hardware, hypa_bringup
```

**示例**：
```
feat: add trajectory planning algorithm

- Implement cubic spline interpolation
- Add velocity and acceleration constraints
- Support waypoint smoothing

affected-packages: hypa_application, hypa_msgs
```
