#ifndef ZMOTION_DRIVER__ZMOTION_SDK_WRAPPER_HPP_
#define ZMOTION_DRIVER__ZMOTION_SDK_WRAPPER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "zmotion_driver/zmotion_types.hpp"

// 若要在多轴绝对移动时使用逐轴的 MOVEMODIFY（`ZAux_Direct_MoveModify`），
// 可在此处启用宏 `ZMOTION_USE_MOVEMODIFY_FOR_MULTIAXIS`。
//
// 注意：`MOVEMODIFY` 是单轴修改接口，且通常用于修改正在执行的运动的目标位置；
// 在没有已有运动上下文时调用可能失败。因此默认保持禁用（注释），
// 仅在明确理解其时序语义并需要按轴修改的场景下启用。
//
// 启用示例（取消下面一行的注释）：
#define ZMOTION_USE_MOVEMODIFY_FOR_MULTIAXIS

namespace zmotion_driver
{

struct CallResult
{
  bool ok = false;
  bool retriable = false;
  int code = -1;
  std::string message;

  static CallResult Success();
  static CallResult Failure(int _code, const std::string &_message,
                            bool _retriable = false);
};

class ZMotionSdkWrapper
{
  public:
  ZMotionSdkWrapper();
  ~ZMotionSdkWrapper();

  ZMotionSdkWrapper(const ZMotionSdkWrapper &) = delete;
  ZMotionSdkWrapper &operator=(const ZMotionSdkWrapper &) = delete;

  CallResult Connect(const std::string &_ip);
  CallResult Disconnect();
  bool IsConnected() const;

  CallResult InitEthercat(const EcatConfig &_config);

  CallResult ConfigureAxis(int _axis, double _units, double _speed,
                           double _accel, double _decel);
  CallResult SetFastDec(int _axis, double _fast_decel);
  CallResult SetAxisEnable(int _axis, bool _enable);

  CallResult CommandVelocity(int _axis, double _velocity,
                             double _deadband = 1e-6);
  CallResult MoveAbsoluteMulti(const std::vector<int> &_axes,
                               const std::vector<double> &_positions);
  CallResult MoveRelativeMulti(const std::vector<int> &_axes,
                               const std::vector<double> &_distances);

  CallResult CancelAxis(int _axis);
  CallResult StopAll();

  CallResult GetMpos(int _axis, double *_value) const;
  CallResult GetDpos(int _axis, double *_value) const;
  CallResult GetMspeed(int _axis, double *_value) const;
  CallResult GetDriveTorque(int _axis, double *_value) const;
  CallResult GetAxisStatus(int _axis, int *_value) const;
  CallResult GetInput(int _io_id, int *_value) const;
  CallResult GetRemainBuffer(int _axis, int *_value) const;

  private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

}  // namespace zmotion_driver

#endif  // ZMOTION_DRIVER__ZMOTION_SDK_WRAPPER_HPP_
