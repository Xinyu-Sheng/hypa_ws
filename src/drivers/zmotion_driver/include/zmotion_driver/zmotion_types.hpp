#ifndef ZMOTION_DRIVER__ZMOTION_TYPES_HPP_
#define ZMOTION_DRIVER__ZMOTION_TYPES_HPP_

#include <array>
#include <string>
#include <vector>

namespace zmotion_driver
{

enum class AxisControlMode
{
  kVelocity = 0,
  kPosition = 1,
};

enum class AxisPositionMode
{
  kAbsolute = 0,
  kRelative = 1,
};

struct AxisConfig
{
  int logical_index = -1;
  int physical_axis = -1;
  std::string joint_name;
  AxisControlMode control_mode = AxisControlMode::kPosition;
  AxisPositionMode position_mode = AxisPositionMode::kAbsolute;
  double zero_offset = 0.0;
  double units = 1.0;
  double speed = 10.0;
  double accel = 100.0;
  double decel = 100.0;
  std::string position_topic;
};

enum class IoTriggerMode
{
  kNone = 0,
  kLevelHigh = 1,
  kLevelLow = 2,
  kRisingEdge = 3,
  kFallingEdge = 4,
  kBothEdges = 5,
};

struct IoInputConfig
{
  int io_id = -1;
  std::string state_topic;
  bool emergency_stop_on_high = false;
  IoTriggerMode trigger_mode = IoTriggerMode::kNone;
};

struct EcatInitInfoSet
{
  int InitStructFlag = 0;

  int LocalAxisId = 0;
  int LocalAxisNum = 0;

  int DriveAxisStart = 0;
  int DriveAxisNum = -1;
  int DriveIoStara = 256;
  int DriveIoSpa = 16;
  int DrivePdoMode[128] = {0};
  int DriveEnable = 0;

  int EcatNodeNum = -1;
  int NodeIoId[128] = {0};
  int NodeAIoId[128] = {0};

  int SysClockMode = 1;
  int DcOffsetFlag[128] = {0};
  float DcOffsetTime[128] = {0.0f};

  int BusRedSwitch = 0;
  int RedSpareSlot = 0;
};

struct EcatConfig
{
  int slot_id = 0;
  int timeout_ms = 10000;
  EcatInitInfoSet init;
};

struct AxisOperation
{
  AxisControlMode control_mode = AxisControlMode::kPosition;
  AxisPositionMode position_mode = AxisPositionMode::kAbsolute;
  int logical_axis = -1;
  double value = 0.0;
};

}  // namespace zmotion_driver

#endif  // ZMOTION_DRIVER__ZMOTION_TYPES_HPP_
