// C++ convenience wrapper for EcatInitInfoSet
#ifndef ZMC432_DRIVER_ECAT_INIT_HPP
#define ZMC432_DRIVER_ECAT_INIT_HPP

#include <array>
#include <string>
#include "zmc432_driver/zmotion_ecat.h"
#include <rclcpp/rclcpp.hpp>

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
    out.DriveIoStara = drive_io_stara;
    out.DriveIoSpa = drive_io_spa;
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
    return out;
  }

  static EcatInitInfo from_node(const rclcpp::Node* node,
                                const std::string& prefix = "ecat")
  {
    EcatInitInfo info;
    node->get_parameter_or(prefix + ".use_defaults", info.use_defaults, true);
    node->get_parameter_or(prefix + ".local_axis_id", info.local_axis_id, 0);
    node->get_parameter_or(prefix + ".local_axis_num", info.local_axis_num, 0);
    node->get_parameter_or(prefix + ".drive_axis_start", info.drive_axis_start,
                           0);
    node->get_parameter_or(prefix + ".drive_axis_num", info.drive_axis_num, -1);
    node->get_parameter_or(prefix + ".drive_io_stara", info.drive_io_stara,
                           256);
    node->get_parameter_or(prefix + ".drive_io_spa", info.drive_io_spa, 16);
    node->get_parameter_or(prefix + ".drive_enable", info.drive_enable, 1);
    node->get_parameter_or(prefix + ".ecat_node_num", info.ecat_node_num, -1);
    node->get_parameter_or(prefix + ".sys_clock_mode", info.sys_clock_mode, 1);
    // arrays are not parameterized individually; advanced users can fill
    // manually
    return info;
  }
};

}  // namespace zmc432_driver

#endif  // ZMC432_DRIVER_ECAT_INIT_HPP
