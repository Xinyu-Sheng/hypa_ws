# HYPA 启动包命名规范

## 核心概念

### `..._bringup` 包
**类比**：电脑的 **"开机 BIOS + 操作系统"** - 让硬件进入可用状态

**用途**：启动整个机器人本体
- 硬件驱动加载和初始化
- 核心系统服务启动
- 基础通信建立

**示例**：
```bash
ros2 launch hypa_bringup robot.launch.py  # 机器人"开机"
```

### `..._launch` 包
**类比**：**"打开应用程序"** - 比如启动浏览器或游戏

**用途**：启动特定功能模块
- 按需加载，独立运行

**示例**：
```bash
ros2 launch hypa_launch do_something.launch.py         # 打开某个应用
```

## 使用顺序

1. **开机**：先运行 `hypa_bringup` 让机器人硬件可用
2. **打开应用**：根据需要运行 `hypa_launch` 中的功能模块

## 项目结构

```
hypa_robot/
├── hypa_bringup/     # 机器人本体启动（开机）
└── hypa_launch/      # 功能模块启动（应用）
```
