# zmotion_driver 轴配置迁移说明（v1 -> v2）

## 变更目标

将 axis 下数组统一为逻辑轴顺序：

- 第 i 项就是逻辑轴 i。
- `axis.logical_indices[i]` 表示逻辑轴 i 对应的物理轴号。

## 语义对比

v1（旧）:

- `logical_indices[physical_axis] = logical_axis`
- `joint_names/zero_offsets/units/directions/speeds/accels/decels/position_topics` 都按物理轴顺序。

v2（新）:

- `logical_indices[logical_axis] = physical_axis`
- 上述数组全部按逻辑轴顺序。

## 参数语义速查

- `[逻辑轴索引]`:
  - `axis.logical_indices`（值是物理轴号）
  - `axis.joint_names`
  - `axis.position_modes`
  - `axis.zero_offsets`
  - `axis.units`
  - `axis.directions`
  - `axis.speeds`
  - `axis.accels`
  - `axis.decels`
  - `axis.position_topics`
- `[逻辑轴编号列表]`:
  - `control.velocity_logical_indices`
  - `control.mimic_group1_logical_indices`
  - `control.mimic_group2_logical_indices`
- `[组索引]`:
  - `axis.mimic_position_topics`（第0项是 group1，第1项是 group2）
- `[非轴索引]`:
  - `io.input_ids`、`io.trigger_modes`（按 IO 顺序）
  - `ecat.init.drive_pdo_mode` 等 EtherCAT 初始化数组（按节点/驱动下标）

## 迁移脚本

脚本路径：

- `scripts/migrate_axis_config_v1_to_v2.py`

使用方式：

```bash
python3 src/drivers/zmotion_driver/scripts/migrate_axis_config_v1_to_v2.py \
  src/drivers/zmotion_driver/config/zmotion_driver.yaml
```

指定输出路径（不覆盖原文件）：

```bash
python3 src/drivers/zmotion_driver/scripts/migrate_axis_config_v1_to_v2.py \
  src/drivers/zmotion_driver/config/zmotion_driver.yaml \
  -o /tmp/zmotion_driver_v2.yaml
```

## 手工校验清单

1. `axis.logical_indices` 是 0..11 的一一映射，无重复、无越界。
2. `joint_names` 下标 i 对应逻辑轴 i。
3. `zero_offsets/directions/units/speeds/accels/decels` 下标 i 对应逻辑轴 i。
4. `position_topics` 下标 i 对应逻辑轴 i。
5. `control.velocity_logical_indices`、`control.mimic_group*_logical_indices` 保持逻辑轴语义不变。

## 运行时验收

1. 启动日志会打印 `logical->physical` 映射。
2. `feedback/axis1/position` 对应逻辑轴 1。
3. 速度控制和 mimic 组命令均下发到配置指定的物理轴。

## 注意事项

- 该改造为破坏性切换，不兼容旧 YAML 语义。
- 旧配置若不迁移，轴映射会被错误解释。
