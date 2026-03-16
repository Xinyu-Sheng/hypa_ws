# ZMotion 运动完成判定（单次 / 连续 / 段进度）

本文档汇总了常见的 **ZMotion/ZMC 控制器** 下判断运动是否完成的所有方法（包括 ZBasic 命令与 C/C++ API），并对比优缺点、推荐程度和典型使用场景。

---

## 1. 术语说明

- **单次运动（Single Move）**：一次指令完成后，运动结束（如 `MOVE`, `LINE`, `CIRCLE` 等）。
- **连续运动（Continuous / Buffering）**：通过缓冲多条运动指令（如 `BUFFERMOVE` + `CONTINUE`）构成的连续轨迹，缓冲耗尽后运动结束。
- **段（Segment / Mark）**：每条运动指令都会生成一个 `MOVE_MARK`，控制器内部按顺序执行，`MOVE_CURMARK` 给出当前执行到哪一个段。

---

## 2. 核心判断“是否走完”的方法（单次和连续都可用）

### ✅ 2.1 硬件空闲状态（最可靠）
- **ZBasic 命令**：`?IF_IDLE(axis)`
- **C/C++ API**：`ZAux_Direct_GetIfIdle(handle, axis, &value)`

> 返回：`0` 表示运动中；`-1` 表示空闲完成。

**优点**
- 不依赖位置计算，直接由控制器状态判断。
- 适用于所有运动模式（单次/连续/插补）。

**缺点/注意**
- 如果控制器驱动存在状态反馈异常（某些模式下可能一直不回空闲），需要另补位置/速度判断。

**推荐程度**：⭐⭐⭐⭐⭐（首选）


### ✅ 2.2 轴状态字（可以判断是否运动、是否报警、是否暂停）
- **ZBasic 命令**：`?AXISSTATUS(axis)`
- **C/C++ API**：`ZAux_Direct_GetAxisStatus(handle, axis, &status)`

**优点**
- 可以同时判断“运动中”与“报警/限位”等异常。

**注意**
- 需要根据具体控制器文档解析位含义，不同型号可能略有不同。

**推荐程度**：⭐⭐⭐⭐

---

## 3. “目标位置 / 缓冲目标位置”查询（适合判断是否到位、目标是否改变）

### ✅ 3.1 `ENDMOVE`（当前指令目标位置）
- **ZBasic 命令**：`?ENDMOVE(axis)`
- **C/C++ API**：`ZAux_Direct_GetEndMove(handle, axis, &value)`

**说明**：返回当前正在执行指令的目标位置（绝对值）。

**注意**：对于 `VMOVE`、`DATUM` 等类型，不一定稳定；会实时变化。

**推荐程度**：⭐⭐⭐（适合作为位置校验辅助）


### ✅ 3.2 `ENDMOVE_BUFFER`（整个缓冲后的最终目标位置）
- **ZBasic 命令**：`?ENDMOVE_BUFFER(axis)`
- **C/C++ API**：`ZAux_Direct_GetEndMoveBuffer(handle, axis, &value)`

**说明**：返回当前缓冲中所有运动指令最终结束时的目标位置。

**注意**
- 对 `VMOVE`、`DATUM`、或使用 `REP_OPTION` 等循环坐标的运动，可能不准确或不断变化。

**推荐程度**：⭐⭐⭐（主要用于转换“相对运动到绝对位置”）


### ✅ 3.3 `REMAIN`（当前运动剩余距离）
- **ZBasic 命令**：`?REMAIN(axis)`
- **C/C++ API**：`ZAux_Direct_GetRemain(handle, axis, &value)`

**说明**：返回当前指令还剩多少距离（单位与坐标一致）。

**推荐程度**：⭐⭐⭐（可用于进度估算，但有抖动）

---

## 4. 连续模式：判断“当前到第几段”“已完成多少段”

连续模式的关键在于“段”的概念：控制器将每条运动命令记录为一个 `MOVE_MARK`（段号），执行时按顺序运行。

### ✅ 4.1 当前执行到第几段：`MOVE_CURMARK`
- **ZBasic 命令**：`?MOVE_CURMARK(axis)`
- **C/C++ API**：`ZAux_Direct_GetMoveCurmark(handle, axis, &mark)`

**说明**：返回当前正在执行的 `MOVE_MARK` 值（即当前段号）。

**优点**
- 精准知道“目前跑到哪条指令了”。

**注意**
- Mark 值一般是自动累加的，一个新的运动指令会将 `MOVE_MARK` +1。
- 如果程序中手动修改 `MOVE_MARK`，则需要与发送指令时的值对应。

**推荐程度**：⭐⭐⭐⭐


### ✅ 4.2 缓冲剩余段数：`MOVES_BUFFERED` + `REMAIN_BUFFER`

| 命令 | 说明 | 适用场景 | 注意事项 | 推荐程度 |
|------|------|---------|----------|----------|
| `?MOVES_BUFFERED(axis)` / `ZAux_Direct_GetMovesBuffered` | 返回当前轴缓冲区内还有多少条运动指令（MOVE）未执行 | 想知道“还剩多少条指令等待执行” | 缓冲区容量有限（如 512） | ⭐⭐⭐⭐ |
| `?REMAIN_BUFFER(1) AXIS(axis)` / `ZAux_Direct_GetRemain_LineBuffer` | 返回“剩余线段数”，仅适用于直线插补（LINE） | 用于线性插补的段计数 | 仅直线有效 | ⭐⭐⭐ |
| `?REMAIN_BUFFER() AXIS(axis)` / `ZAux_Direct_GetRemain_Buffer` | 返回“剩余段数（最复杂曲线）” | 复杂插补（圆弧、螺旋等） | 对插补模式敏感，需参考文档说明 | ⭐⭐⭐ |

> ⚠️ 推荐组合：`MOVE_CURMARK` + `MOVES_BUFFERED` / `REMAIN_BUFFER` 来做“已完成/剩余”进度展示。

---

## 5. 使用建议（实战最佳实践）

### 🔧 5.1 单次运动（单条 MOVE/插补）
- **优先**：`?IF_IDLE`（空闲）来判断结束。
- **次选**：若需要更强鲁棒，可再用 `?ENDMOVE` + `?REMAIN` 辅助确认。

### 🔧 5.2 连续运动（缓冲模式）
- **判断“整体结束”**：
  1. 读取 `MOVES_BUFFERED`/`REMAIN_BUFFER` 为 0
  2. 再确认 `?IF_IDLE` 表示轴已经空闲

- **判断“完成一段/当前段”**：
  - 读 `MOVE_CURMARK`（当前段）
  - 读 `MOVES_BUFFERED`（剩余段数）

### 🔧 5.3 不同方法的组合（最稳）
- 进度估算：`MOVE_CURMARK` + `MOVES_BUFFERED` 组合出“已执行=当前段 - 起始段”
- 终点判断：当 `MOVES_BUFFERED==0` 且 `?IF_IDLE` 返回空闲时，认为全部完成

---

## 6. 代码层面（zmc432_driver）现状说明

- 当前 `motion_controller.cpp` 的 `is_motion_complete()` 方法：
  - 优先用 `is_axis_idle()`（`?IF_IDLE`）判断
  - 失败时，退回到位置/速度容差判断（`Feedback` + `Speed`）
  - **没有**使用 `MOVE_CURMARK` / `MOVES_BUFFERED` / `REMAIN_BUFFER` 等段级查询

> 如果你希望支持“连续模式的段级完成回调/进度”，可以扩展 `ZMotionWrapper` 加入对应接口（如 `get_moves_buffered()` / `get_move_curmark()`），并在 `MotionController` 里利用上面列出的查询方式。

---

## 7. 其他常见注意点

- **`ENDMOVE_BUFFER` 在循环坐标（REP_OPTION）时会随 `REP_DIST` 改变**，谨慎使用。
- **缓冲区满（512+）会导致后续 `MOVE` 阻塞**，必须在发送运动前读取 `MOVES_BUFFERED`、`REMAIN_BUFFER` 做流控。
- **`AXISSTATUS` 里还有暂停/报警状态**，若需监控异常、暂停、限位，一定要解析状态位。

---

> 需要我把上述“段级进度查询”直接加到现有 driver 里（例如在 `ZMotionWrapper` 增加 `get_move_curmark()` / `get_moves_buffered()` 接口，并在 `MotionController` 里输出进度）请告诉我，我可以帮你写具体实现。