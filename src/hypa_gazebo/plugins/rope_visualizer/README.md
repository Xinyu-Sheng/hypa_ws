# Rope Visualizer Plugin

## 概述
RopeVisualizer 插件用于在 Gazebo Sim 中动态可视化绳索或缆绳。它通过在两个链节之间绘制一个**圆柱体**来实时显示绳索的长度变化，并可选择显示端点。

## 功能
- 实时绘制两个链节之间的**圆柱体**绳索
- 可选择显示端点球体标记
- 支持自定义链节名称、绳索直径、颜色和端点显示
- 基于链节世界姿态自动更新绳索位置和方向

## 使用方法

### 1. 编译插件
```bash
cd /home/xinyu/Projects/HKU/hypa_ws
colcon build --packages-select hypa_gazebo
source install/setup.bash
```

### 2. 在 SDF 模型中添加插件
在你的模型 SDF 文件中添加以下插件配置：

```xml
<model name="your_model_name">
  <!-- 你的模型内容，包括两个链节 -->
  
  <link name="start_link">
    <!-- 起始链节定义 -->
  </link>
  
  <link name="end_link">
    <!-- 结束链节定义 -->
  </link>
  
  <!-- 添加绳索可视化插件 -->
  <plugin
    filename="libRopeVisualizer.so"
    name="hypa_gazebo::systems::RopeVisualizer">
    <start_link>base_link</start_link>
    <end_link>end_link</end_link>
    <line_width>0.1</line_width>
    <color>
      <r>1.0</r>
      <g>0.0</g>
      <b>0.0</b>
      <a>1.0</a>
    </color>
    <show_endpoints>true</show_endpoints>
  </plugin>
</model>
```

### 3. 参数说明
- `start_link`: 绳索起始链节名称（默认: "start_link"）
- `end_link`: 绳索结束链节名称（默认: "end_link"）
- `line_width`: 绳索直径（米，默认: 0.02，现已支持）
- `color`: 标记颜色（RGBA，默认: 红色）
  - `r`: 红色分量 (0.0-1.0，默认: 1.0)
  - `g`: 绿色分量 (0.0-1.0，默认: 0.0)
  - `b`: 蓝色分量 (0.0-1.0，默认: 0.0)
  - `a`: 透明度 (0.0-1.0，默认: 1.0)
- `show_endpoints`: 是否显示端点球体标记（默认: true）

### 4. 运行仿真
```bash
# 使用提供的示例
gz sim /home/xinyu/Projects/HKU/hypa_ws/src/hypa_gazebo/worlds/rope_example.sdf

# 或者使用你自己的世界文件
gz sim your_world.sdf
```

### 5. 测试绳索动态变化
在 Gazebo GUI 中：
1. 使用 Joint Control 插件或通过命令控制棱柱关节
2. 观察**圆柱体绳索**和端点标记实时更新显示绳索长度变化
3. **圆柱体**会自动连接 start_link 和 end_link 的当前位置并调整方向

## 示例场景
已提供示例文件 `rope_example.sdf`，包含：
- 蓝色 start_link（固定在 z=1m）
- 红色 end_link（通过棱柱关节连接）
- 可调节的绳索长度（0-5米）
- 实时**绿色圆柱体绳索**（直径 0.1 米）和端点球体可视化

## 注意事项
- 插件必须附加到模型上，不能附加到世界或链节
- 确保指定的链节名称在模型中存在
- 如果链节未找到，会在控制台输出警告信息
- **圆柱体**为纯视觉效果，不影响物理仿真

## 适用场景
- 起重机缆绳可视化
- 拖拽系统绳索显示
- 机器人绳索驱动系统
- 任何需要动态显示连接关系的场景
