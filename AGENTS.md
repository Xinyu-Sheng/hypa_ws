# AI Agent Instructions for HYPA Workspace

## 📋 目录

- [环境要求](#环境要求)
- [核心要求](#核心要求)
- [构建系统](#构建系统)
- [依赖管理](#依赖管理)
- [代码规范](#代码规范)
- [Commit 规范](#commit-规范)
- [目标](#目标)

---

## 环境要求

**重要**：在生成或修改代码前，必须确认当前工作区的 ROS 2 版本。

确认方法：
```bash
ros2 --version
```
所有生成的代码必须与所确认的 ROS 2 版本兼容。注释和文档用中文。

---

## 核心要求

所有AI代理必须生成**可构建、可维护、可扩展的ROS 2代码**

### 强制要求

生成的代码必须：
- ✅ 成功构建（`colcon build`）
- ✅ 无伪代码、假API、缺失头文件
- ✅ 仅使用ROS 2（禁止ROS 1概念混合）
- ✅ 遵守ROS 2包边界

### 节点参数支持

所有节点必须支持：
- `namespace` - 命名空间
- `use_sim_time` - 仿真时钟
- `robot_name` - 机器人名称

### 禁止项

⛔ 禁止假设：
- 单机器人系统
- 根命名空间主题
- 伪代码/不可构建代码
- 省略包含/发明API
- ROS 1概念混合
- 忽略包边界

---

## 构建系统

### 必须参数

所有构建命令**必须**包含以下 CMake 参数以生成 `compile_commands.json`（Clangd 支持）：

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

---

## 依赖管理

### ROS 依赖

通过 `find_package()` 找到的包，使用 `ament_target_dependencies()`：

```cmake
ament_target_dependencies(target rclcpp hypa_msgs ...)
```

优点：自动处理 include、链接、编译定义

### 非 ROS 依赖

第三方库（如 ZMotion SDK），使用 `target_link_libraries()`：

```cmake
target_link_libraries(target zmotion)
```

### 项目头文件

项目自己的 include 路径，使用 `target_include_directories()`：

```cmake
target_include_directories(target PUBLIC include)
```

---

## 代码规范

详见 [CONTRIBUTING.md](./CONTRIBUTING.md)

关键点：
- 使用 `this->` 访问类属性和成员函数
- 函数参数以 `_` 下划线开头
- 大括号 `{}` 独占一行
- 使用前缀 `++i` 而非 `i++`
- 新类必须使用 PIMPL 模式
- 不修改成员变量的函数标记为 `const`
- 非 POD 参数标记为 `const`
- `*` 和 `&` 紧邻变量名（`int& variable`）

---

## Commit 规范

**重要**：所有 commit 基于暂存区（staged）内容。内容简洁，使用双引号，自然换行。

### 格式

```
<类型>: <简要描述>

<详细说明>

affected-packages: pkg1, pkg2
```

### 类型

| 类型       | 说明                       |
| ---------- | -------------------------- |
| `feat`     | 新功能                     |
| `fix`      | 修复 bug                   |
| `docs`     | 文档变更                   |
| `style`    | 代码格式调整（不影响功能） |
| `refactor` | 代码重构                   |
| `test`     | 测试相关                   |
| `chore`    | 构建/工具变更              |

### 示例

```
feat: add trajectory planning algorithm

- Implement cubic spline interpolation
- Add velocity and acceleration constraints
- Support waypoint smoothing

affected-packages: hypa_application, hypa_msgs
```

禁止自动暂存，commit内容要精简。

---

## 目标

确保所有AI生成的ROS 2代码：

- ✅ **可构建** - 成功编译，无隐藏依赖
- ✅ **可维护** - 代码清晰，符合规范
- ✅ **多机器人安全** - 支持命名空间、参数化
- ✅ **工具兼容** - 支持 clangd、colcon、ros2_control

## 额外注意事项
package.xml 中 maintainer是：
<maintainer email="sheng.xin.yu@faxmail.com">Xinyu Sheng</maintainer>

不要修改：
libzmotion.so  zmcaux.cpp  zmcaux.h  zmotion.h
上面这些文件。

最终用中文回答我。