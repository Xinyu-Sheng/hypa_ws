/**
 * @file ecat_init.hpp
 * @brief EtherCAT 初始化流程的 C++ 包装器
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * EtherCAT 主控制器（ZMC432-V2）与从站驱动器（AKD）初始化的 11 步流程
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * 📊 各步骤的实际目标对象与数据流向
 *
 * | 步骤 | 操作函数 | 目标对象 | 被改主体 | 执行层级 | 说明 |
 * |------|---------|---------|---------|--------|------|
 * | [1] | ZAux_Direct_Rapidstop + 轴参数重置 | 控制器 | 虚拟轴配置、运动缓冲 |
 * 控制器内核 | 紧急停止→清空旧运动指令 | | [2] | ZAux_BusCmd_SlotScan |
 * 控制器+EtherCAT总线 | 总线设备表 | 主站扫描 | 扫描网络上所有从站设备 | | [3]
 * | ZAux_Direct_SetAxisAddress | 控制器 | 轴→驱动器映射表 | 控制器虚拟轴 |
 * 绑定逻辑轴到物理从站 | | [4] | ZAux_Direct_SetAtype | 控制器 | 轴类型标志
 * (ATYPE) | 控制器虚拟轴 | 定义为脉冲轴/EtherCAT轴 | | [5] | DRIVE_PROFILE 配置
 * | **驱动器从站** | PDO数据结构 | 从站内部参数 | 定义PDO映射→实时数据格式 | |
 * [6] | DRIVE_IO + SETREVIN/SETFWDIN | **驱动器从站** | I/O方向标志 |
 * 从站内部参数 | 配置正反向IO映射 | | [7] | SLOT_START (OP模式) | EtherCAT总线
 * | 实时通信状态 | 主站+从站 | 切换为OP模式→开始循环交互 | | [8] | WDOG = 1 |
 * 控制器 | 看门狗使能标志 | 控制器安全模块 | 工业安全保护→检测心跳 | | [9] |
 * DRIVE_CONTROLWORD + 故障清除 | **驱动器从站** | 状态机控制字 | 从站状态机 |
 * 清驱动器故障→进入Ready | | [10] | ZAux_Direct_Single_Datum | 控制器 |
 * 错误标志、软件计数器 | 控制器内核 | 复位控制器内部软件错误 | | [11] |
 * ZAux_Direct_SetAxisEnable | 控制器 | 轴使能标志 | 控制器虚拟轴 |
 * 电机通电→准备运动 |
 *
 * 🔄 核心区分——三层控制关系
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │ PC 主程序                                                         │
 * └────────────────────┬────────────────────────────────────────────┘
 *                      │ USB/网络 (ZMotion私有协议)
 *                      ↓
 * ┌─────────────────────────────────────────────────────────────────┐
 * │ ZMC432-V2 主控制器 (EtherCAT主站)                                │
 * │                                                                   │
 * │ 步骤[1,3,4,8,10,11]: 配置控制器虚拟轴 (ZAux_Direct_*API)         │
 * │ - 轴寻址、类型设置、使能控制                                     │
 * │ - 直接修改控制器内部的轴配置表                                   │
 * └──────────────────┬─────────────────────────────────────────────┘
 *                    │ EtherCAT总线 (250μs~1ms周期)
 *                    ↓
 * ┌─────────────────────────────────────────────────────────────────┐
 * │ Kollmorgen AKD 驱动器从站 (0,1,2,...)                           │
 * │                                                                   │
 * │ 步骤[2,5,6,9]: 配置驱动器内部参数 (PDO/SDO/控制字)               │
 * │ - 控制器通过EtherCAT循环发送→驱动器状态机执行                    │
 * │ - 驱动器通过TxPDO反馈实际位置、状态                            │
 * │ - DRIVE_PROFILE: 定义RxPDO(接收)和TxPDO(反馈)的映射          │
 * │ - DRIVE_CONTROLWORD: 状态机命令字                              │
 * └─────────────────────────────────────────────────────────────────┘
 *                    ↑
 *                    │ EtherCAT Sync0信号 (DC时钟)
 *            所有从站锁定到同一时钟源
 *
 * 📋 数据流向总结
 *
 * 控制器配置阶段 ([1-4]步)
 *   ↓
 *   PC主程序 ──→ ZMC432 ──→ 控制器虚拟轴表
 *                ↓
 *              虚拟轴[0-31]与EtherCAT从站[0-N]建立映射关系
 *
 * 驱动器初始化阶段 ([5-9]步)
 *   ↓
 *   ZMC432 ──(EtherCAT)──→ AKD驱动器PDO ──→ 驱动器内部参数
 *                  ↑━━━━━━━━━━━━━━↑
 *            TxPDO反馈（位置/速度/状态）
 *
 * 运行准备阶段 ([10-11]步)
 *   ↓
 *   控制器错误复位
 *   │
 *   └─→ 轴使能开启 ──→ 电机通电 ──→ 进入运动控制循环
 *
 * ⚠️ 关键理解点
 *
 * 1. 步骤[1,3,4,8,10,11]是"控制器内配置"：
 *    - 操作对象：ZMC432 内的虚拟轴表、参数表
 *    - 这些配置定义了"控制器应该如何理解这个轴"
 *    - 通过 ZAux_Direct_* API 直接调用
 *
 * 2. 步骤[5,6,9]是"驱动器内配置"：
 *    - 操作对象：AKD 驱动器内的参数、状态机
 *    - 通过 EtherCAT 网络循环地发送命令字和参数
 *    - 驱动器执行→反馈状态字
 *
 * 3. 步骤[2,7]是"总线管理"：
 *    - 步骤[2]：扫描总线上有多少驱动器节点（动态检测）
 *    - 步骤[7]：启动EtherCAT实时循环（从PreOp→OP模式）
 *
 * 4. DC 时钟同步（分布式时钟）：
 *    - 所有从站通过Sync0信号锁定到ZMC432发出的时钟
 *    - 时钟精度影响多轴同步误差（目标<1μs）
 *
 * 5. 实时性保证：
 *    - 步骤[7]启动后，EtherCAT循环以固定周期（通常250μs）运行
 *    - PC端通过缓冲区机制发送轨迹命令，控制器保证实时执行
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 */

#ifndef ZMC432_DRIVER_ECAT_INIT_HPP
#define ZMC432_DRIVER_ECAT_INIT_HPP

#include <array>
#include <string>

#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "zmc432_driver/zmotion_ecat.h"

namespace zmc432_driver
{

struct EcatInitInfo
{
  bool use_defaults = true;
  int local_axis_id = 0;
  int local_axis_num = 0;
  int drive_axis_start = 0;
  int drive_axis_num = -1;
  int drive_io_stara = 256;
  int drive_io_spa = 16;
  std::array<int, 128> drive_pdo_mode{};
  int drive_enable = 1;
  int ecat_node_num = -1;
  std::array<int, 128> node_io_id{};
  std::array<int, 128> node_aio_id{};
  int sys_clock_mode = 1;
  std::array<int, 128> dc_offset_flag{};
  std::array<float, 128> dc_offset_time{};
  int watchdog_time = 1;

  EcatInitInfo()
  {
    // default values match those in the Windows sample comments:
    drive_pdo_mode.fill(12);
    node_io_id.fill(0);
    node_aio_id.fill(0);
    dc_offset_flag.fill(0);
    dc_offset_time.fill(0.0f);
  }

  EcatInitInfoSet toC() const
  {
    EcatInitInfoSet out;
    out.InitStructFlag = use_defaults ? 1 : 0;
    out.LocalAxisId = local_axis_id;
    out.LocalAxisNum = local_axis_num;
    out.DriveAxisStart = drive_axis_start;
    out.DriveAxisNum = drive_axis_num;
    out.DriveIoStara = drive_io_stara;  // 实际上是DriveIoStartAddress
    out.DriveIoSpa = drive_io_spa;      // 实际上是DriveIoSpan
    for (size_t i = 0; i < drive_pdo_mode.size(); ++i)
    {
      out.DrivePdoMode[i] = drive_pdo_mode[i];
    }
    out.DriveEnable = drive_enable;
    out.EcatNodeNum = ecat_node_num;
    for (size_t i = 0; i < node_io_id.size(); ++i)
    {
      out.NodeIoId[i] = node_io_id[i];
      out.NodeAIoId[i] = node_aio_id[i];
    }
    out.SysClockMode = sys_clock_mode;
    for (size_t i = 0; i < dc_offset_flag.size(); ++i)
    {
      out.DcOffsetFlag[i] = dc_offset_flag[i];
      out.DcOffsetTime[i] = dc_offset_time[i];
    }
    out.WatchDogTime = watchdog_time;
    return out;
  }

  static EcatInitInfo from_node(
      rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params,
      const std::string &prefix = "ecat")
  {
    EcatInitInfo info;
    info.use_defaults = node_params
                            ->declare_parameter(prefix + ".use_defaults",
                                                rclcpp::ParameterValue(true))
                            .get<bool>();
    info.local_axis_id = node_params
                             ->declare_parameter(prefix + ".local_axis_id",
                                                 rclcpp::ParameterValue(0))
                             .get<int>();
    info.local_axis_num = node_params
                              ->declare_parameter(prefix + ".local_axis_num",
                                                  rclcpp::ParameterValue(0))
                              .get<int>();
    info.drive_axis_start =
        node_params
            ->declare_parameter(prefix + ".drive_axis_start",
                                rclcpp::ParameterValue(0))
            .get<int>();
    info.drive_axis_num = node_params
                              ->declare_parameter(prefix + ".drive_axis_num",
                                                  rclcpp::ParameterValue(-1))
                              .get<int>();
    info.drive_io_stara = node_params
                              ->declare_parameter(prefix + ".drive_io_stara",
                                                  rclcpp::ParameterValue(256))
                              .get<int>();
    info.drive_io_spa = node_params
                            ->declare_parameter(prefix + ".drive_io_spa",
                                                rclcpp::ParameterValue(16))
                            .get<int>();
    info.drive_enable = node_params
                            ->declare_parameter(prefix + ".drive_enable",
                                                rclcpp::ParameterValue(1))
                            .get<int>();
    info.ecat_node_num = node_params
                             ->declare_parameter(prefix + ".ecat_node_num",
                                                 rclcpp::ParameterValue(-1))
                             .get<int>();
    info.sys_clock_mode = node_params
                              ->declare_parameter(prefix + ".sys_clock_mode",
                                                  rclcpp::ParameterValue(1))
                              .get<int>();
    // arrays are not parameterized individually; advanced users can fill
    // manually
    return info;
  }
};

}  // namespace zmc432_driver

#endif  // ZMC432_DRIVER_ECAT_INIT_HPP
