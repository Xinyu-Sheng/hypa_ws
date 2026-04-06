#include "zmotion_driver/zmotion_sdk_wrapper.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "zmcaux.h"
#include "zmotion.h"

namespace zmotion_driver
{
namespace
{

constexpr int kAckBufferSize = 2048;
constexpr int kErrOk = 0;
constexpr int kErrNoAck = 30000;

constexpr int kWrongNodeNum = -1;
constexpr int kWrongAxisNum = -2;
constexpr int kNotScanNode = -3;
constexpr int kEcatStartFailed = -4;

bool IsRetriableCode(const int _code)
{
  return (_code == kErrNoAck) || (_code == 20003) || (_code == 3402);
}

std::string TrimString(const std::string &_input)
{
  if (_input.empty())
  {
    return "";
  }

  std::size_t start = 0;
  std::size_t end = _input.size();

  while ((start < end) && ((_input[start] == ' ') || (_input[start] == '\n') ||
                           (_input[start] == '\r') || (_input[start] == '\t') ||
                           (_input[start] == '\0')))
  {
    ++start;
  }

  while ((end > start) &&
         ((_input[end - 1] == ' ') || (_input[end - 1] == '\n') ||
          (_input[end - 1] == '\r') || (_input[end - 1] == '\t') ||
          (_input[end - 1] == '\0')))
  {
    --end;
  }

  return _input.substr(start, end - start);
}

CallResult WrapCode(const int _code, const std::string &_context)
{
  if (_code == kErrOk)
  {
    return CallResult::Success();
  }

  std::ostringstream oss;
  oss << _context << " (code=" << _code << ")";
  return CallResult::Failure(_code, oss.str(), IsRetriableCode(_code));
}

}  // namespace

CallResult CallResult::Success()
{
  CallResult result;
  result.ok = true;
  result.retriable = false;
  result.code = 0;
  result.message = "ok";
  return result;
}

CallResult CallResult::Failure(const int _code, const std::string &_message,
                               const bool _retriable)
{
  CallResult result;
  result.ok = false;
  result.retriable = _retriable;
  result.code = _code;
  result.message = _message;
  return result;
}

class ZMotionSdkWrapper::Impl
{
  public:
  ZMC_HANDLE handle = nullptr;
  bool connected = false;
  std::unordered_set<int> moving_axes;

  CallResult Execute(const std::string &_command, std::string *_response) const
  {
    if (!this->connected || (this->handle == nullptr))
    {
      return CallResult::Failure(-100, "controller not connected", false);
    }

    char ack[kAckBufferSize] = {0};
    const int32 code =
        ZAux_Execute(this->handle, _command.c_str(), ack, kAckBufferSize);
    if (code != kErrOk)
    {
      std::ostringstream oss;
      oss << "execute failed: " << _command;
      return CallResult::Failure(code, oss.str(), IsRetriableCode(code));
    }

    if (_response != nullptr)
    {
      *_response = TrimString(std::string(ack));
    }
    return CallResult::Success();
  }

  CallResult DirectCommand(const std::string &_command,
                           std::string *_response) const
  {
    if (!this->connected || (this->handle == nullptr))
    {
      return CallResult::Failure(-100, "controller not connected", false);
    }

    char ack[kAckBufferSize] = {0};
    const int32 code =
        ZAux_DirectCommand(this->handle, _command.c_str(), ack, kAckBufferSize);
    if (code != kErrOk)
    {
      std::ostringstream oss;
      oss << "direct command failed: " << _command;
      return CallResult::Failure(code, oss.str(), IsRetriableCode(code));
    }

    if (_response != nullptr)
    {
      *_response = TrimString(std::string(ack));
    }
    return CallResult::Success();
  }

  CallResult WaitSlotResponse(const std::string &_expected,
                              const int _timeout_ms) const
  {
    if (!this->connected || (this->handle == nullptr))
    {
      return CallResult::Failure(-100, "controller not connected", false);
    }

    int elapsed_ms = 0;
    while (elapsed_ms <= _timeout_ms)
    {
      char receive_buffer[1000] = {0};
      uint32 read_size = 0;
      uint8 execute_done = 0;
      const int32 code = ZMC_ExecuteGetReceive(this->handle, receive_buffer,
                                               sizeof(receive_buffer),
                                               &read_size, &execute_done);

      if ((code == kErrOk) || IsRetriableCode(code))
      {
        const std::string response = TrimString(std::string(receive_buffer));
        if (!response.empty())
        {
          if (response == _expected)
          {
            return CallResult::Success();
          }
          return CallResult::Failure(kEcatStartFailed,
                                     "unexpected slot response: " + response +
                                         " expected: " + _expected,
                                     false);
        }
      }
      else
      {
        return CallResult::Failure(code, "poll slot response failed",
                                   IsRetriableCode(code));
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      elapsed_ms += 50;
    }

    return CallResult::Failure(kNotScanNode, "wait slot response timeout",
                               true);
  }

  CallResult SlotScan(const int _slot_id, const int _timeout_ms,
                      const int _bus_red_switch,
                      const int _red_spare_slot) const
  {
    // 停止槽位
    (void)this->Execute("SLOT_STOP(" + std::to_string(_slot_id) + ")", nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 冗余设置（仅在启用冗余时写入）
    if ((_bus_red_switch == 1) && (_slot_id != _red_spare_slot))
    {
      const std::string slave_cmd = "SLOT_SLAVE(" + std::to_string(_slot_id) +
                                    ")=" + std::to_string(_red_spare_slot);
      const CallResult slave_result = this->Execute(slave_cmd, nullptr);
      if (!slave_result.ok)
      {
        return slave_result;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::string response;
    const std::string scan_cmd =
        "SLOT_SCAN(" + std::to_string(_slot_id) + ") ?return";
    const CallResult scan_result = this->Execute(scan_cmd, &response);
    if (!scan_result.ok)
    {
      return scan_result;
    }
    if (response == "-1")
    {
      return CallResult::Success();
    }

    return this->WaitSlotResponse("-1", _timeout_ms);
  }

  CallResult StartSlotOp(const int _slot_id, const int _timeout_ms) const
  {
    const CallResult safe_op_result = this->Execute(
        "SLOT_START(" + std::to_string(_slot_id) + ", 4)", nullptr);
    if (!safe_op_result.ok)
    {
      return safe_op_result;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::string response;
    const CallResult start_result = this->Execute(
        "SLOT_START(" + std::to_string(_slot_id) + ", 8) ?return", &response);
    if (!start_result.ok)
    {
      return start_result;
    }

    if (response == "-1")
    {
      return CallResult::Success();
    }

    return this->WaitSlotResponse("-1", _timeout_ms);
  }

  CallResult QueryNodeCount(const int _slot_id, int *_node_count) const
  {
    if (_node_count == nullptr)
    {
      return CallResult::Failure(-101, "node_count pointer is null", false);
    }

    std::string response;
    const CallResult result = this->Execute(
        "?NODE_COUNT(" + std::to_string(_slot_id) + ")", &response);
    if (!result.ok)
    {
      return result;
    }

    try
    {
      *_node_count = std::stoi(response);
    }
    catch (const std::exception &)
    {
      return CallResult::Failure(
          -102, "invalid NODE_COUNT response: " + response, false);
    }
    return CallResult::Success();
  }

  CallResult QueryNodeAxisCount(const int _slot_id, const int _node_id,
                                int *_axis_count) const
  {
    if (_axis_count == nullptr)
    {
      return CallResult::Failure(-103, "axis_count pointer is null", false);
    }

    std::string response;
    const std::string cmd = "?NODE_AXIS_COUNT(" + std::to_string(_slot_id) +
                            "," + std::to_string(_node_id) + ")";
    const CallResult result = this->Execute(cmd, &response);
    if (!result.ok)
    {
      return result;
    }

    try
    {
      *_axis_count = std::stoi(response);
    }
    catch (const std::exception &)
    {
      return CallResult::Failure(
          -104, "invalid NODE_AXIS_COUNT response: " + response, false);
    }
    return CallResult::Success();
  }

  CallResult QueryNodeInfo(const int _slot_id, const int _node_id,
                           const int _selector, int *_value) const
  {
    if (_value == nullptr)
    {
      return CallResult::Failure(-105, "node_info value pointer is null",
                                 false);
    }

    std::string response;
    const std::string cmd = "?NODE_INFO(" + std::to_string(_slot_id) + "," +
                            std::to_string(_node_id) + ", " +
                            std::to_string(_selector) + ")";
    const CallResult result = this->Execute(cmd, &response);
    if (!result.ok)
    {
      return result;
    }

    try
    {
      *_value = std::stoi(response);
    }
    catch (const std::exception &)
    {
      return CallResult::Failure(
          -106, "invalid NODE_INFO response: " + response, false);
    }
    return CallResult::Success();
  }
};

ZMotionSdkWrapper::ZMotionSdkWrapper() : pimpl_(std::make_unique<Impl>())
{
}

ZMotionSdkWrapper::~ZMotionSdkWrapper()
{
  (void)this->Disconnect();
}

CallResult ZMotionSdkWrapper::Connect(const std::string &_ip)
{
  if (this->pimpl_->connected)
  {
    return CallResult::Success();
  }

  static bool linux_lib_inited = false;
  if (!linux_lib_inited)
  {
    (void)ZMC_LinuxLibInit();
    linux_lib_inited = true;
  }

  ZMC_HANDLE handle = nullptr;
  const int32 code = ZAux_OpenEth(const_cast<char *>(_ip.c_str()), &handle);
  if (code != kErrOk)
  {
    return CallResult::Failure(code, "ZAux_OpenEth failed: " + _ip,
                               IsRetriableCode(code));
  }

  this->pimpl_->handle = handle;
  this->pimpl_->connected = true;
  (void)ZAux_SetTimeOut(this->pimpl_->handle, 1000);

  return CallResult::Success();
}

CallResult ZMotionSdkWrapper::Disconnect()
{
  if (!this->pimpl_->connected)
  {
    return CallResult::Success();
  }

  const int32 code = ZAux_Close(this->pimpl_->handle);
  this->pimpl_->connected = false;
  this->pimpl_->handle = nullptr;
  this->pimpl_->moving_axes.clear();
  return WrapCode(code, "ZAux_Close failed");
}

bool ZMotionSdkWrapper::IsConnected() const
{
  return this->pimpl_->connected;
}

CallResult ZMotionSdkWrapper::InitEthercat(const EcatConfig &_config)
{
  if (!this->pimpl_->connected)
  {
    return CallResult::Failure(-110, "controller not connected", false);
  }

  const EcatInitInfoSet &cfg = _config.init;

  // 1) 还原虚拟轴，避免遗留映射干扰。
  uint16 virtual_axis_count = 0;
  uint8 max_motor = 0;
  uint8 max_io = 0;
  const int32 spec_code = ZAux_GetSysSpecification(
      this->pimpl_->handle, &virtual_axis_count, &max_motor, &max_io);
  if (spec_code != kErrOk)
  {
    return WrapCode(spec_code, "ZAux_GetSysSpecification failed");
  }

  (void)ZAux_Direct_Rapidstop(this->pimpl_->handle, 2);

  for (int i = 0; i < static_cast<int>(virtual_axis_count); ++i)
  {
    const int32 addr_code =
        ZAux_Direct_SetAxisAddress(this->pimpl_->handle, i, 0);
    if (addr_code != kErrOk)
    {
      return WrapCode(addr_code, "ZAux_Direct_SetAxisAddress reset failed");
    }

    const int32 enable_code =
        ZAux_Direct_SetAxisEnable(this->pimpl_->handle, i, 0);
    if (enable_code != kErrOk)
    {
      return WrapCode(enable_code, "ZAux_Direct_SetAxisEnable reset failed");
    }

    const int32 atype_code = ZAux_Direct_SetAtype(this->pimpl_->handle, i, 0);
    if (atype_code != kErrOk)
    {
      return WrapCode(atype_code, "ZAux_Direct_SetAtype reset failed");
    }
  }

  // 2) 本地脉冲轴映射。
  for (int i = 0; i < cfg.LocalAxisNum; ++i)
  {
    const int axis = cfg.LocalAxisId + i;
    const int32 addr_code =
        ZAux_Direct_SetAxisAddress(this->pimpl_->handle, axis, (-65536 + i));
    if (addr_code != kErrOk)
    {
      return WrapCode(addr_code, "set local axis address failed");
    }

    const int32 atype_code =
        ZAux_Direct_SetAtype(this->pimpl_->handle, axis, 1);
    if (atype_code != kErrOk)
    {
      return WrapCode(atype_code, "set local axis type failed");
    }
  }

  // 3) DC 同步时钟开关。
  if (cfg.SysClockMode == 1)
  {
    const CallResult result =
        this->pimpl_->Execute("SYSTEM_ZSET = SET_BIT(7, SYSTEM_ZSET)", nullptr);
    if (!result.ok)
    {
      return result;
    }
  }
  else
  {
    const CallResult result = this->pimpl_->Execute(
        "SYSTEM_ZSET = CLEAR_BIT(7, SYSTEM_ZSET)", nullptr);
    if (!result.ok)
    {
      return result;
    }
  }

  // 4) 首次扫描并写入 DC 偏移。
  {
    CallResult scan_result =
        this->pimpl_->SlotScan(_config.slot_id, _config.timeout_ms,
                               cfg.BusRedSwitch, cfg.RedSpareSlot);
    if (!scan_result.ok)
    {
      return scan_result;
    }

    int node_count = 0;
    CallResult node_result =
        this->pimpl_->QueryNodeCount(_config.slot_id, &node_count);
    if (!node_result.ok)
    {
      return node_result;
    }

    for (int node = 0; node < node_count; ++node)
    {
      if (cfg.DcOffsetFlag[node] == 0)
      {
        continue;
      }

      int vendor = 0;
      int device = 0;
      CallResult vendor_result =
          this->pimpl_->QueryNodeInfo(_config.slot_id, node, 0, &vendor);
      if (!vendor_result.ok)
      {
        return vendor_result;
      }
      CallResult device_result =
          this->pimpl_->QueryNodeInfo(_config.slot_id, node, 1, &device);
      if (!device_result.ok)
      {
        return device_result;
      }

      std::ostringstream cmd;
      cmd << "ZML_INFO(19, " << vendor << ", " << device
          << ") = SERVO_PERIOD * " << cfg.DcOffsetTime[node] << " * 1000";
      CallResult dc_result = this->pimpl_->Execute(cmd.str(), nullptr);
      if (!dc_result.ok)
      {
        return dc_result;
      }
    }
  }

  // 5) 二次扫描。
  {
    CallResult scan_result =
        this->pimpl_->SlotScan(_config.slot_id, _config.timeout_ms,
                               cfg.BusRedSwitch, cfg.RedSpareSlot);
    if (!scan_result.ok)
    {
      return scan_result;
    }
  }

  // 6) 节点数与轴数校验。
  int node_count = 0;
  {
    CallResult node_result =
        this->pimpl_->QueryNodeCount(_config.slot_id, &node_count);
    if (!node_result.ok)
    {
      return node_result;
    }

    if ((cfg.EcatNodeNum >= 0) && (node_count != cfg.EcatNodeNum))
    {
      return CallResult::Failure(kWrongNodeNum, "ecat node count mismatch",
                                 false);
    }
  }

  if (cfg.DriveAxisNum >= 0)
  {
    int sum_axis = 0;
    for (int node = 0; node < node_count; ++node)
    {
      int node_axis = 0;
      CallResult axis_result =
          this->pimpl_->QueryNodeAxisCount(_config.slot_id, node, &node_axis);
      if (!axis_result.ok)
      {
        return axis_result;
      }
      sum_axis += node_axis;
    }

    if (sum_axis != cfg.DriveAxisNum)
    {
      return CallResult::Failure(kWrongAxisNum,
                                 "ecat drive axis count mismatch", false);
    }
  }

  // 7) 轴映射、PDO 与 IO 映射。
  int bus_axis_num = 0;
  for (int node = 0; node < node_count; ++node)
  {
    int node_axis = 0;
    CallResult axis_result =
        this->pimpl_->QueryNodeAxisCount(_config.slot_id, node, &node_axis);
    if (!axis_result.ok)
    {
      return axis_result;
    }

    if (cfg.NodeIoId[node] >= 32)
    {
      std::ostringstream cmd;
      cmd << "NODE_IO(" << _config.slot_id << ", " << node
          << ") = " << cfg.NodeIoId[node];
      CallResult map_result = this->pimpl_->Execute(cmd.str(), nullptr);
      if (!map_result.ok)
      {
        return map_result;
      }
    }

    if (cfg.NodeAIoId[node] > 0)
    {
      std::ostringstream cmd;
      cmd << "NODE_AIO(" << _config.slot_id << ", " << node
          << ") = " << cfg.NodeAIoId[node];
      CallResult map_result = this->pimpl_->Execute(cmd.str(), nullptr);
      if (!map_result.ok)
      {
        return map_result;
      }
    }

    for (int local = 0; local < node_axis; ++local)
    {
      const int axis = cfg.DriveAxisStart + bus_axis_num;
      const int axis_address = bus_axis_num + 1 + (_config.slot_id << 16);

      const int32 addr_code =
          ZAux_Direct_SetAxisAddress(this->pimpl_->handle, axis, axis_address);
      if (addr_code != kErrOk)
      {
        return WrapCode(addr_code, "set bus axis address failed");
      }

      const int32 atype_code =
          ZAux_Direct_SetAtype(this->pimpl_->handle, axis, 65);
      if (atype_code != kErrOk)
      {
        return WrapCode(atype_code, "set bus axis type failed");
      }

      const int pdo_mode = cfg.DrivePdoMode[bus_axis_num];
      {
        std::ostringstream cmd;
        cmd << "DRIVE_PROFILE(" << cfg.DriveAxisStart << " + " << bus_axis_num
            << ") = " << pdo_mode;
        CallResult pdo_result = this->pimpl_->Execute(cmd.str(), nullptr);
        if (!pdo_result.ok)
        {
          return pdo_result;
        }
      }

      if ((pdo_mode == 4) || (pdo_mode == 5) || (pdo_mode == 12))
      {
        const int io_start = cfg.DriveIoStara + cfg.DriveIoSpa * bus_axis_num;
        std::ostringstream cmd;
        cmd << "DRIVE_IO(" << cfg.DriveAxisStart << " + " << bus_axis_num
            << ") = " << io_start;
        CallResult io_result = this->pimpl_->Execute(cmd.str(), nullptr);
        if (!io_result.ok)
        {
          return io_result;
        }
      }

      {
        std::ostringstream cmd;
        cmd << "DISABLE_GROUP(" << axis << ")";
        CallResult group_result = this->pimpl_->Execute(cmd.str(), nullptr);
        if (!group_result.ok)
        {
          return group_result;
        }
      }

      ++bus_axis_num;
    }
  }

  // 8) 启动总线到 OP。
  {
    CallResult start_result =
        this->pimpl_->StartSlotOp(_config.slot_id, _config.timeout_ms);
    if (!start_result.ok)
    {
      return CallResult::Failure(kEcatStartFailed, start_result.message,
                                 start_result.retriable);
    }
  }

  // 9) 清报警并打开 WDOG；是否使能由配置决定。
  for (int axis = cfg.DriveAxisStart;
       axis < (cfg.DriveAxisStart + bus_axis_num); ++axis)
  {
    (void)this->pimpl_->Execute(
        "DRIVE_CONTROLWORD(" + std::to_string(axis) + ")=128", nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    (void)this->pimpl_->Execute(
        "DRIVE_CONTROLWORD(" + std::to_string(axis) + ")=6", nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    (void)this->pimpl_->Execute(
        "DRIVE_CONTROLWORD(" + std::to_string(axis) + ")=15", nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    (void)ZAux_BusCmd_DriveClear(this->pimpl_->handle, axis, 0);
  }

  {
    CallResult wdog_result = this->pimpl_->Execute("WDOG=1", nullptr);
    if (!wdog_result.ok)
    {
      return wdog_result;
    }
  }

  if (cfg.DriveEnable == 1)
  {
    for (int axis = cfg.DriveAxisStart;
         axis < (cfg.DriveAxisStart + bus_axis_num); ++axis)
    {
      const int32 code =
          ZAux_Direct_SetAxisEnable(this->pimpl_->handle, axis, 1);
      if (code != kErrOk)
      {
        return WrapCode(code, "set axis enable during ecat init failed");
      }
    }
  }

  return CallResult::Success();
}

CallResult ZMotionSdkWrapper::ConfigureAxis(const int _axis,
                                            const double _units,
                                            const double _speed,
                                            const double _accel,
                                            const double _decel)
{
  const int32 units_code = ZAux_Direct_SetUnits(this->pimpl_->handle, _axis,
                                                static_cast<float>(_units));
  if (units_code != kErrOk)
  {
    return WrapCode(units_code, "ZAux_Direct_SetUnits failed");
  }

  const int32 speed_code = ZAux_Direct_SetSpeed(this->pimpl_->handle, _axis,
                                                static_cast<float>(_speed));
  if (speed_code != kErrOk)
  {
    return WrapCode(speed_code, "ZAux_Direct_SetSpeed failed");
  }

  const int32 accel_code = ZAux_Direct_SetAccel(this->pimpl_->handle, _axis,
                                                static_cast<float>(_accel));
  if (accel_code != kErrOk)
  {
    return WrapCode(accel_code, "ZAux_Direct_SetAccel failed");
  }

  const int32 decel_code = ZAux_Direct_SetDecel(this->pimpl_->handle, _axis,
                                                static_cast<float>(_decel));
  if (decel_code != kErrOk)
  {
    return WrapCode(decel_code, "ZAux_Direct_SetDecel failed");
  }

  return CallResult::Success();
}

CallResult ZMotionSdkWrapper::SetAxisEnable(const int _axis, const bool _enable)
{
  const int32 code =
      ZAux_Direct_SetAxisEnable(this->pimpl_->handle, _axis, _enable ? 1 : 0);
  return WrapCode(code, "ZAux_Direct_SetAxisEnable failed");
}

CallResult ZMotionSdkWrapper::CommandVelocity(const int _axis,
                                              const double _velocity,
                                              const double _deadband)
{
  if (std::fabs(_velocity) <= _deadband)
  {
    const int32 cancel_code =
        ZAux_Direct_Single_Cancel(this->pimpl_->handle, _axis, 2);
    const CallResult cancel_result =
        WrapCode(cancel_code, "ZAux_Direct_Single_Cancel failed");
    if (cancel_result.ok)
    {
      this->pimpl_->moving_axes.erase(_axis);
    }
    return cancel_result;
  }

  const int32 speed_code = ZAux_Direct_SetSpeed(this->pimpl_->handle, _axis,
                                                static_cast<float>(_velocity));
  if (speed_code != kErrOk)
  {
    return WrapCode(speed_code, "ZAux_Direct_SetSpeed failed");
  }

  if (this->pimpl_->moving_axes.find(_axis) != this->pimpl_->moving_axes.end())
  {
    return CallResult::Success();
  }

  const int direction = (_velocity > 0.0) ? 1 : -1;
  const int32 move_code =
      ZAux_Direct_Single_Vmove(this->pimpl_->handle, _axis, direction);
  const CallResult move_result =
      WrapCode(move_code, "ZAux_Direct_Single_Vmove failed");
  if (move_result.ok)
  {
    this->pimpl_->moving_axes.insert(_axis);
  }
  return move_result;
}

CallResult ZMotionSdkWrapper::MoveAbsoluteMulti(
    const std::vector<int> &_axes, const std::vector<double> &_positions)
{
  if (_axes.empty() || (_axes.size() != _positions.size()))
  {
    return CallResult::Failure(-120, "invalid absolute move array sizes",
                               false);
  }

  std::vector<int> axis_buffer = _axes;
  std::vector<float> pos_buffer;
  pos_buffer.reserve(_positions.size());
  for (std::size_t i = 0; i < _positions.size(); ++i)
  {
    pos_buffer.push_back(static_cast<float>(_positions[i]));
  }

  const int32 code =
      ZAux_Direct_MoveAbs(this->pimpl_->handle, static_cast<int>(_axes.size()),
                          axis_buffer.data(), pos_buffer.data());
  return WrapCode(code, "ZAux_Direct_MoveAbs failed");
}

CallResult ZMotionSdkWrapper::MoveRelativeMulti(
    const std::vector<int> &_axes, const std::vector<double> &_distances)
{
  if (_axes.empty() || (_axes.size() != _distances.size()))
  {
    return CallResult::Failure(-121, "invalid relative move array sizes",
                               false);
  }

  std::vector<int> axis_buffer = _axes;
  std::vector<float> distance_buffer;
  distance_buffer.reserve(_distances.size());
  for (std::size_t i = 0; i < _distances.size(); ++i)
  {
    distance_buffer.push_back(static_cast<float>(_distances[i]));
  }

  const int32 code =
      ZAux_Direct_Move(this->pimpl_->handle, static_cast<int>(_axes.size()),
                       axis_buffer.data(), distance_buffer.data());
  return WrapCode(code, "ZAux_Direct_Move failed");
}

CallResult ZMotionSdkWrapper::CancelAxis(const int _axis)
{
  const int32 code = ZAux_Direct_Single_Cancel(this->pimpl_->handle, _axis, 2);
  const CallResult result = WrapCode(code, "ZAux_Direct_Single_Cancel failed");
  if (result.ok)
  {
    this->pimpl_->moving_axes.erase(_axis);
  }
  return result;
}

CallResult ZMotionSdkWrapper::StopAll()
{
  const int32 code = ZAux_Direct_Rapidstop(this->pimpl_->handle, 2);
  const CallResult result = WrapCode(code, "ZAux_Direct_Rapidstop failed");
  if (result.ok)
  {
    this->pimpl_->moving_axes.clear();
  }
  return result;
}

CallResult ZMotionSdkWrapper::GetMpos(const int _axis, double *_value) const
{
  if (_value == nullptr)
  {
    return CallResult::Failure(-130, "GetMpos output pointer is null", false);
  }

  float value = 0.0f;
  const int32 code = ZAux_Direct_GetMpos(this->pimpl_->handle, _axis, &value);
  if (code != kErrOk)
  {
    return WrapCode(code, "ZAux_Direct_GetMpos failed");
  }

  *_value = static_cast<double>(value);
  return CallResult::Success();
}

CallResult ZMotionSdkWrapper::GetDpos(const int _axis, double *_value) const
{
  if (_value == nullptr)
  {
    return CallResult::Failure(-131, "GetDpos output pointer is null", false);
  }

  float value = 0.0f;
  const int32 code = ZAux_Direct_GetDpos(this->pimpl_->handle, _axis, &value);
  if (code != kErrOk)
  {
    return WrapCode(code, "ZAux_Direct_GetDpos failed");
  }

  *_value = static_cast<double>(value);
  return CallResult::Success();
}

CallResult ZMotionSdkWrapper::GetMspeed(const int _axis, double *_value) const
{
  if (_value == nullptr)
  {
    return CallResult::Failure(-132, "GetMspeed output pointer is null", false);
  }

  float value = 0.0f;
  const int32 code = ZAux_Direct_GetMspeed(this->pimpl_->handle, _axis, &value);
  if (code != kErrOk)
  {
    return WrapCode(code, "ZAux_Direct_GetMspeed failed");
  }

  *_value = static_cast<double>(value);
  return CallResult::Success();
}

CallResult ZMotionSdkWrapper::GetDriveTorque(const int _axis,
                                             double *_value) const
{
  if (_value == nullptr)
  {
    return CallResult::Failure(-136, "GetDriveTorque output pointer is null",
                               false);
  }

  int32 value = 0;
  const int32 code =
      ZAux_BusCmd_GetDriveTorque(this->pimpl_->handle, _axis, &value);
  if (code != kErrOk)
  {
    return WrapCode(code, "ZAux_BusCmd_GetDriveTorque failed");
  }

  *_value = static_cast<double>(value);
  return CallResult::Success();
}

CallResult ZMotionSdkWrapper::GetAxisStatus(const int _axis, int *_value) const
{
  if (_value == nullptr)
  {
    return CallResult::Failure(-133, "GetAxisStatus output pointer is null",
                               false);
  }

  int32 value = 0;
  const int32 code =
      ZAux_Direct_GetAxisStatus(this->pimpl_->handle, _axis, &value);
  if (code != kErrOk)
  {
    return WrapCode(code, "ZAux_Direct_GetAxisStatus failed");
  }

  *_value = static_cast<int>(value);
  return CallResult::Success();
}

CallResult ZMotionSdkWrapper::GetInput(const int _io_id, int *_value) const
{
  if (_value == nullptr)
  {
    return CallResult::Failure(-134, "GetInput output pointer is null", false);
  }

  uint32 value = 0;
  const int32 code = ZAux_Direct_GetIn(this->pimpl_->handle, _io_id, &value);
  if (code != kErrOk)
  {
    return WrapCode(code, "ZAux_Direct_GetIn failed");
  }

  *_value = static_cast<int>(value);
  return CallResult::Success();
}

CallResult ZMotionSdkWrapper::GetRemainBuffer(const int _axis,
                                              int *_value) const
{
  if (_value == nullptr)
  {
    return CallResult::Failure(-135, "GetRemainBuffer output pointer is null",
                               false);
  }

  int32 value = 0;
  const int32 code =
      ZAux_Direct_GetRemain_Buffer(this->pimpl_->handle, _axis, &value);
  if (code != kErrOk)
  {
    return WrapCode(code, "ZAux_Direct_GetRemain_Buffer failed");
  }

  *_value = static_cast<int>(value);
  return CallResult::Success();
}

}  // namespace zmotion_driver
