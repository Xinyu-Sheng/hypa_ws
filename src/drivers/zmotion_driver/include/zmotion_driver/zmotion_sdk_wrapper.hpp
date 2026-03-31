#ifndef ZMOTION_DRIVER__ZMOTION_SDK_WRAPPER_HPP_
#define ZMOTION_DRIVER__ZMOTION_SDK_WRAPPER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "zmotion_driver/zmotion_types.hpp"

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
  CallResult GetAxisStatus(int _axis, int *_value) const;
  CallResult GetInput(int _io_id, int *_value) const;
  CallResult GetRemainBuffer(int _axis, int *_value) const;

  private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

}  // namespace zmotion_driver

#endif  // ZMOTION_DRIVER__ZMOTION_SDK_WRAPPER_HPP_
