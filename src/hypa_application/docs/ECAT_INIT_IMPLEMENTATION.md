# EtherCAT 初始化实现细节指南

**关联文档**：`solution.md`（总体架构）  
**目标文件**：`src/drivers/zmc432_driver/src/ecat_init.cpp`  
**版本**：V1.0  
**日期**：2026-03-08

---

## 1. 11步初始化流程——完整映射表

### 1.1 目标对象与数据流向

| 步骤     | 操作函数                       | 目标对象       | 被改主体             | 执行层级     | 关键参数                             |
| -------- | ------------------------------ | -------------- | -------------------- | ------------ | ------------------------------------ |
| **[1]**  | `ZAux_Direct_Rapidstop()`      | 控制器         | 虚拟轴配置、运动缓冲 | 控制器内核   | `handle`                             |
|          | 轴参数重置循环                 | 控制器         | 轴类型、速度、位置   | 虚拟轴[0-31] | `ZAux_Direct_SetAtype(handle, i, 0)` |
| **[2]**  | `ZAux_BusCmd_EcatInit()`       | 控制器+总线    | 总线设备表           | 主站扫描     | `SlotId=0`                           |
| **[3]**  | `ZAux_Direct_SetAxisAddress()` | 控制器         | 轴→驱动器映射表      | 虚拟轴       | `iaxis, slave_index`                 |
| **[4]**  | `ZAux_Direct_SetAtype()`       | 控制器         | 轴类型标志 (ATYPE)   | 虚拟轴       | `iaxis, ATYPE=65(EtherCAT)`          |
| **[5]**  | `DRIVE_PROFILE_Config_Loop`    | **驱动器从站** | PDO数据结构          | 从站参数     | `PDO映射、RxPDO、TxPDO`              |
| **[6]**  | `DRIVE_IO_Mapping`             | **驱动器从站** | I/O方向标志          | 从站参数     | `SETREVIN、SETFWDIN`                 |
| **[7]**  | `SLOT_START(SlotId, Mode)`     | EtherCAT总线   | 实时通信状态         | 主站+从站    | `Mode=8(OP模式)`                     |
| **[8]**  | `WDOG = 1`                     | 控制器         | 看门狗使能标志       | 安全模块     | `看门狗计数器`                       |
| **[9]**  | `DRIVE_CONTROLWORD_Config`     | **驱动器从站** | 状态机控制字         | 从站状态机   | `0x80→0x06→0x0F`                     |
| **[10]** | `ZAux_Direct_Single_Datum()`   | 控制器         | 错误标志、计数器     | 内核         | `iaxis=0, datum=0`                   |
| **[11]** | `ZAux_Direct_SetAxisEnable()`  | 控制器         | 轴使能标志           | 虚拟轴       | `iaxis, enable=1`                    |

---

## 2. 三层控制关系详解

### 2.1 架构图

```
┌─────────────────────────────────────────────────────────────────┐
│ PC 主程序 (C#/C++)                                               │
│ • 复杂数学运算（冗余优化、逆解）                                 │
│ • PVT 轨迹生成与下发                                            │
└────────────────────┬────────────────────────────────────────────┘
                     │ USB/网络 (ZMotion私有协议)
                     └──→ handle (连接号)
                     
┌─────────────────────────────────────────────────────────────────┐
│ ZMC432-V2 主控制器 (EtherCAT主站)                                │
│                                                                   │
│ 【步骤 1,3,4,8,10,11】控制器虚拟轴配置                            │
│ • ZAux_Direct_* API 直接修改控制器内部的轴表                      │
│ • 定义：轴类型、地址映射、使能状态                               │
│ • 范围：轴 0-31 (ZMC432 最多 32 轴)                              │
│ • 结果：虚拟轴与EtherCAT从站建立映射关系                          │
│                                                                   │
│ 【步骤 2,7】EtherCAT 总线管理                                    │
│ • 扫描总线→检测从站数量                                          │
│ • 启动 OP 模式→开启实时循环（250μs-1ms周期）                    │
│                                                                   │
│ 【步骤 8】看门狗保护                                             │
│ • 心跳检测→检查系统健康状态                                      │
│                                                                   │
└──────────────────┬─────────────────────────────────────────────┘
                   │ EtherCAT 总线 (星形/链式拓扑)
                   │ SM0 = Master → All Slaves (RxPDO)
                   │ SM1 = All Slaves → Master (TxPDO)
                   │ DC Sync_0 = 时钟同步信号
                   ↓
┌─────────────────────────────────────────────────────────────────┐
│ Kollmorgen AKD 驱动器从站 (ID=0,1,2,...)                        │
│                                                                   │
│ 【步骤 5,6,9】驱动器内部参数配置                                  │
│ • PDO 映射：定义与主站的数据交互格式                             │
│   - RxPDO(从主站接收)：目标位置、速度、控制字                   │
│   - TxPDO(反馈给主站)：实际位置、速度、状态字、故障码            │
│ • DRIVE_PROFILE：定义PDO的具体映射编号                          │
│ • DRIVE_CONTROLWORD：状态机命令（0x80→0x06→0x0F→Ready→OP）  │
│ • DRIVE_IO：定义I/O极性（正反方向）                             │
│                                                                   │
│ • 注意：这些参数不是直接修改驱动器，而是通过EtherCAT                │
│   循环的 SDO(Service Data Object) 或 CoE(CANopen over EtherCAT)  │
│   接口发给驱动器，驱动器状态机执行变更                           │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. 数据流向分层详解

### 3.1 【步骤 1-4】控制器配置阶段

```
PC 主程序
  │
  └─→ ZAux_Direct_Rapidstop(handle)
      └─ 紧急停止所有轴
  
  └─→ 循环[i=0...31]: ZAux_Direct_SetAtype(handle, i, 0)
      └─ 清空旧轴配置（ATYPE=0表示禁用）
  
  └─→ 循环[i=0...N]: 
      ├─ ZAux_Direct_SetAxisAddress(handle, iaxis, i)
      │  └─ 轴 iaxis 的地址 = EtherCAT从站索引 i
      │
      └─ ZAux_Direct_SetAtype(handle, iaxis, 65)  // 65 = EtherCAT伺服轴
         └─ 定义轴类型 = "EtherCAT模式"

┌─────────────────────────────┐
│ 控制器内部轴表 (AxisTable)    │
│                             │
│ iaxis=0: {                  │
│   ATYPE = 65 (EtherCAT)     │
│   ADDRESS = 0 (从站ID)      │
│   状态 = "配置中"           │
│ }                           │
│ ...                         │
└─────────────────────────────┘
```

**关键点**：  
- `ZAux_Direct_SetAxisAddress()` 完成的是逻辑轴 → 物理从站的映射
- 控制器不知道 AKD 的 PDO 格式，只知道"这个轴连在从站 i"
- PDO 格式由步骤[5-6]的 DRIVE_PROFILE 定义

---

### 3.2 【步骤 5-6】驱动器 PDO 配置阶段

```
ZMC432 主站
  │
  └─→ 循环[i=0...N]（每个从站）:
      │
      ├─ DRIVE_PROFILE(从站i) = PDO_MODE
      │  说明：告诉驱动器将使用哪个PDO映射方案
      │  如：DRIVE_PROFILE = 12（预定义的PDO组合）
      │
      ├─ SETREVIN(从站i) = 反向输入极性  ┐
      │                                  ├─ DRIVE_IO 映射
      ├─ SETFWDIN(从站i) = 正向输入极性  ┘
      │
      └─ [EtherCAT RxPDO 循环] 
         └─ 每个周期（250μs）发送实时位置/速度目标给驱动器

┌──────────────────────────────────┐
│ AKD 驱动器 (从站 i)               │
│                                  │
│ 接收 DRIVE_PROFILE = 12：         │
│   → 启用 RxPDO 映射组合 #12      │
│   → 准备在 RxPDO 接收：          │
│      • 目标位置 (Target Pos)    │
│      • 目标速度 (Target Vel)    │
│      • 控制字 (Control Word)    │
│                                  │
│ TxPDO 反馈（每周期自动）：        │
│   • 实际位置 (Actual Pos)       │
│   • 实际速度 (Actual Vel)       │
│   • 状态字 (Status Word)        │
│   • 故障码 (Fault Code)         │
└──────────────────────────────────┘
```

**关键概念**：
- **PDO（Process Data Object）**：EtherCAT 中的实时循环数据包
- **RxPDO**：从站接收方向（主站→从站）
- **TxPDO**：从站发送方向（从站→主站）
- **DRIVE_PROFILE = 12**：预定义的PDO映射编号，驱动器和控制器都认识

---

### 3.3 【步骤 9】驱动器状态机控制阶段

```
ZMC432 主站
  │
  └─→ 驱动器状态机初始化循环：
      │
      ├─ DRIVE_CONTROLWORD = 0x80  (Drive Fault Clear)
      │  └─ 清除驱动器内部故障标志
      │
      ├─ DRIVE_CONTROLWORD = 0x06  (Shutdown Command)
      │  └─ 驱动器进入 "Ready to Switch On" 状态
      │
      └─ DRIVE_CONTROLWORD = 0x0F  (Operation Enabled)
         └─ 驱动器进入 "Operation Enabled" 状态

┌──────────────────────────────────────┐
│ AKD 驱动器状态机 (根据控制字转移)      │
│                                      │
│ 初始状态：Fault                      │
│    ↓ (收到 0x80)                    │
│ 状态：Not Ready to Switch On         │
│    ↓ (收到 0x06)                    │
│ 状态：Ready to Switch On             │
│    ↓ (收到 0x0F)                    │
│ 状态：Operation Enabled ✓           │
│    ↓ (收到轴运动命令)                 │
│ 执行：电机旋转、位置控制            │
└──────────────────────────────────────┘
```

**约定**：
- `DRIVE_CONTROLWORD` 通过 EtherCAT RxPDO 循环地发送给驱动器
- 驱动器每个周期(250μs)接收一次新的控制字
- 驱动器状态机按CANopen DSP-402标准实现

---

### 3.4 【步骤 7】EtherCAT 实时循环启动

```
ZMC432 主站
  │
  ├─ SLOT_START(SlotId=0, Mode=4)  // PreOp → Init
  │  └─ 初始化阶段
  │
  ├─ SLOT_START(SlotId=0, Mode=8)  // → Op
  │  └─ 启动实时循环（250μs周期）
  │
  └─────→ 无限循环：
          ├─ 第 1 周期 (250μs):
          │  ├─ 读取 PC 端 ZMotion 缓冲区的轨迹指令
          │  ├─ 触发 EtherCAT Sync_0 时钟
          │  ├─ 发送 RxPDO (目标位置/速度/控制字) → 所有从站
          │  ├─ 接收 TxPDO (实际位置/速度/状态) ← 所有从站
          │  └─ 更新控制器内部的轴反馈值
          │
          ├─ 第 2 周期 (250μs):
          │  └─ (重复)
          │
          └─ ...（直到 stop 或故障）

┌─────────────────────────────────────┐
│ DC 时钟同步（分布式时钟）             │
│                                     │
│ Master(ZMC432) 发送 Sync_0 信号     │
│         │                          │
│         ├─→ Slave_0 (AKD) 锁定    │
│         ├─→ Slave_1 (AKD) 锁定    │
│         ├─→ Slave_2 (AKD) 锁定    │
│         └─→ ... (所有从站同步)      │
│                                     │
│ 效果：所有驱动器以同一时间基准执行   │
│      多轴同步误差 < 1μs 保证        │
└─────────────────────────────────────┘
```

**关键参数**：
- `SlotId = 0`：第一个 EtherCAT插槽（ZMC432通常只有1个）
- `Mode = 8`：OP(Operational) 模式，开启实时循环
- `周期 = 250μs`：通常配置，可根据需求改为 500μs/1ms

---

## 4. 控制器配置 vs 驱动器配置的区别

### 4.1 什么是"控制器配置"？

**步骤 [1,3,4,8,10,11]**

- **操作对象**：ZMC432-V2 内部的软件配置表
- **API接口**：`ZAux_Direct_*()` 函数族
- **特点**：
  - 直接调用（不经过EtherCAT总线）
  - 响应快速（通常 <1ms）
  - 操作对象是控制器虚拟轴的属性
  - 不直接控制硬件，只是告诉控制器"这个轴的配置是什么"

```cpp
// 例子：控制器配置
ZAux_Direct_SetAxisAddress(handle, iaxis, slave_index);
// 效果：控制器内部 AxisTable[iaxis].address = slave_index;
//      不会立即发送给驱动器，只是记录配置
```

### 4.2 什么是"驱动器配置"？

**步骤 [5,6,9]**

- **操作对象**：AKD 驱动器内部的参数、状态机
- **传输机制**：EtherCAT PDO 循环数据
- **特点**：
  - 通过EtherCAT网络发送（非直接API）
  - 响应由驱动器执行（可能有延迟）
  - 驱动器接收后更新自己的内部状态
  - 是"主站 → 从站"的单向命令

```cpp
// 例子：驱动器配置
DRIVE_PROFILE(Drivei) = 12;
// 效果：下一个EtherCAT周期，此值被编码到RxPDO中发送给驱动器
//      驱动器收到后，改变自己的PDO映射配置
//      主站在 TxPDO 中接收驱动器的反馈
```

---

## 5. 关键参数与对象字典

### 5.1 AKD 对象字典（CANopen DSP-402）

| 对象                   | 地址     | 功能                              | 步骤        |
| ---------------------- | -------- | --------------------------------- | ----------- |
| **Modes of Operation** | 0x6060h  | 设置工作模式                      | [5]         |
| 值=8                   | -        | Cyclic Synchronous Position (CSP) | 必须        |
| **DRIVE_PROFILE**      | 应用定义 | PDO映射编号                       | [5]         |
| 值=12                  | -        | 预定义映射组合                    | solution.md |
| **DRIVE_CONTROLWORD**  | 应用定义 | 状态机控制字                      | [9]         |
| 0x80                   | -        | Drive Fault Clear                 | 清故障      |
| 0x06                   | -        | Shutdown                          | Ready       |
| 0x0F                   | -        | Operation Enabled                 | OP          |
| **RxPDO**              | -        | 主站→从站数据                     | [7]         |
| 组成                   | -        | 目标位置、速度、控制字            | 每250μs     |
| **TxPDO**              | -        | 从站→主站反馈                     | [7]         |
| 组成                   | -        | 实际位置、速度、状态字            | 每250μs     |

### 5.2 ZMC432内部参数

| 参数    | ZAux函数           | 功能             | 步骤 |
| ------- | ------------------ | ---------------- | ---- |
| ATYPE   | `SetAtype()`       | 轴类型 (0/1/65)  | [4]  |
| ADDRESS | `SetAxisAddress()` | 轴→从站映射      | [3]  |
| ENABLE  | `SetAxisEnable()`  | 轴使能           | [11] |
| WDOG    | 直接赋值           | 看门狗计数       | [8]  |
| FE      | `GetFe()`          | 跟随误差（整数） | 监测 |
| ERROR   | `Single_Datum()`   | 错误清除         | [10] |

---

## 6. 实现检查清单

### 6.1 需要实现的代码

```cpp
// ecat_init.cpp 中应包含的函数

// ✅ 已实现
int ecat_init_step1_rapidstop()
{
  // [1] 紧急停止 + 轴参数重置
  ZAux_Direct_Rapidstop(handle);
  for(int i = 0; i < 32; i++) {
    ZAux_Direct_SetAtype(handle, i, 0);  // 清空旧配置
  }
}

// ❌ 需要补齐的关键步骤

int ecat_init_step5_drive_profile()
{
  // [5] DRIVE_PROFILE 配置
  // 问题：当前代码无此实现，缺失对驱动器PDO映射的配置
  for(int i = 0; i < num_slaves; i++) {
    int Drivei = i;
    // TODO: 设置 DRIVE_PROFILE(Drivei) = 12
    // 可能需要：ZAux_BusCmd_SDOWrite(handle, i, 0x????, 12);
  }
}

int ecat_init_step9_drive_controlword()
{
  // [9] 驱动器故障清除 + 状态机初始化
  for(int i = 0; i < num_slaves; i++) {
    int Drivei = i;
    // TODO: DRIVE_CONTROLWORD(Drivei) = 0x80;  // 故障清除
    // TODO: DRIVE_CONTROLWORD(Drivei) = 0x06;  // Shutdown
    // TODO: DRIVE_CONTROLWORD(Drivei) = 0x0F;  // Operation Enabled
  }
}

int ecat_init_step7_slot_start_op()
{
  // [7] 启动EtherCAT OP 模式
  // 问题：当前代码可能有，但需验证是否正确设置为Mode=8
  int SlotId = 0;
  SLOT_START(SlotId, 8);  // 8 = OP mode
}
```

### 6.2 关键函数缺失表

| 函数                         | 当前状态 | 需要补齐            | 优先级 |
| ---------------------------- | -------- | ------------------- | ------ |
| `ZAux_Direct_MultiMovePvt`   | ❌ 缺失   | **PVT轨迹下发核心** | P0     |
| `ZAux_BusCmd_SDOWrite`       | ?        | 驱动器参数补齐      | P1     |
| `SLOT_START(..., 8)`         | ? 需验证 | OP模式启动          | P1     |
| `ZAux_Direct_SetAxisAddress` | ✅ 存在   | -                   | -      |
| `ZAux_Direct_SetAtype`       | ✅ 存在   | -                   | -      |
| `ZAux_Direct_SetAxisEnable`  | ✅ 存在   | -                   | -      |

---

## 7. 与 solution.md 的映射关系

| solution.md 段落         | 对应步骤 | 实现位置                             |
| ------------------------ | -------- | ------------------------------------ |
| §3.1 工作模式选择（CSP） | [5]      | DRIVE_PROFILE 配置                   |
| §3.1 分布式时钟(DC)同步  | [7]      | SLOT_START OP模式                    |
| §3.2 EtherCAT主站初始化  | [2]      | ZAux_BusCmd_EcatInit                 |
| §3.2 轴参数映射          | [3,4]    | SetAxisAddress/SetAtype              |
| §3.3 PVT数据下发         | -        | **ZAux_Direct_MultiMovePvt** (缺失!) |
| §4.2 缓冲区耗尽风险      | -        | 需在PC端实现缓冲管理                 |

---

## 8. 调试与验证步骤

### 8.1 单步验证

```bash
# Step 1: 验证控制器连接
./zmc_test_connection

# Step 2: 验证EtherCAT总线
./zmc_test_ecat_scan
# 应输出：Found N slaves

# Step 3: 验证轴配置
./zmc_test_axis_config
# 应输出：iaxis=0 -> ADDRESS=0, ATYPE=65

# Step 4: 验证PDO映射
./zmc_test_pdo_mapping
# 应输出：RxPDO映射完成, TxPDO映射完成

# Step 5: 验证DC同步
./zmc_test_dc_sync
# 应输出：Sync0对齐, Drift < 1μs

# Step 6: 验证驱动器状态
./zmc_test_drive_status
# 应输出：所有驱动器 = Operation Enabled

# Step 7: 验证多轴同步
./zmc_test_multi_axis_sync
# 应输出：相位误差 < 1μs
```

### 8.2 常见问题

| 现象                  | 可能原因                | 对策                              |
| --------------------- | ----------------------- | --------------------------------- |
| 总线扫描失败          | 网线未连接、驱动器断电  | 检查EtherCAT网线、电源、LED指示灯 |
| 驱动器卡在"Not Ready" | 步骤[9]的控制字序列错误 | 检查 0x80→0x06→0x0F 顺序          |
| 多轴不同步            | DC偏移参数未配置        | 检查步骤[7]的DC sync开启          |
| PVT轨迹断流           | PC端缓冲区耗尽          | 增加缓冲深度、优化轨迹计算        |

---

## 9. 后续工作

### 9.1 立即需要（P0）

- [ ] 验证 `ZAux_Direct_MultiMovePvt` 是否在 zm caux.h 中存在
- [ ] 如不存在，从老版本参考代码中提取实现
- [ ] 在 zmotion_wrapper.cpp 中封装 PVT 下发接口

### 9.2 需要补齐（P1）

- [ ] 补齐步骤[5]的 DRIVE_PROFILE 配置
- [ ] 补齐步骤[9]的 DRIVE_CONTROLWORD 状态机
- [ ] 验证步骤[7]的 SLOT_START 是否正确使用 Mode=8

### 9.3 集成测试（P2）

- [ ] 编写单元测试覆盖 11 步流程
- [ ] 进行硬件集成测试（实际 AKD 驱动器）
- [ ] 测试多轴同步精度

---

**文档维护**：如有更新，请同步修改 [ecat_init.hpp](../../../drivers/zmc432_driver/include/zmc432_driver/ecat_init.hpp) 的头部注释。
