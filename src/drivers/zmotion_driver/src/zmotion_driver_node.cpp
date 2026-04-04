#include "zmotion_driver/zmotion_driver_node.hpp"

#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "zmotion_driver/zmotion_sdk_wrapper.hpp"
#include "zmotion_driver/zmotion_types.hpp"

namespace zmotion_driver
{
namespace
{

constexpr std::size_t kAxisCount = 9;

std::vector<int> ConvertToIntVector(const std::vector<int64_t> &_input)
{
  std::vector<int> output;
  output.reserve(_input.size());
  for (std::size_t i = 0; i < _input.size(); ++i)
  {
    output.push_back(static_cast<int>(_input[i]));
  }
  return output;
}

std::string JoinInts(const std::vector<int> &_values)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < _values.size(); ++i)
  {
    if (i > 0)
    {
      oss << ";";
    }
    oss << _values[i];
  }
  return oss.str();
}

std::string JoinDoubles(const std::vector<double> &_values)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < _values.size(); ++i)
  {
    if (i > 0)
    {
      oss << ";";
    }
    oss << _values[i];
  }
  return oss.str();
}

std::string JoinStrings(const std::vector<std::string> &_values)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < _values.size(); ++i)
  {
    if (i > 0)
    {
      oss << ";";
    }
    oss << _values[i];
  }
  return oss.str();
}

std::string SanitizeCsv(const std::string &_text)
{
  std::string output = _text;
  for (std::size_t i = 0; i < output.size(); ++i)
  {
    if ((output[i] == ',') || (output[i] == '\n') || (output[i] == '\r'))
    {
      output[i] = ';';
    }
  }
  return output;
}

bool ParseControlMode(const std::string &_mode, AxisControlMode *_result)
{
  if (_result == nullptr)
  {
    return false;
  }

  if ((_mode == "velocity") || (_mode == "vel"))
  {
    *_result = AxisControlMode::kVelocity;
    return true;
  }
  if ((_mode == "position") || (_mode == "pos"))
  {
    *_result = AxisControlMode::kPosition;
    return true;
  }
  return false;
}

bool ParsePositionMode(const std::string &_mode, AxisPositionMode *_result)
{
  if (_result == nullptr)
  {
    return false;
  }

  if ((_mode == "absolute") || (_mode == "abs"))
  {
    *_result = AxisPositionMode::kAbsolute;
    return true;
  }
  if ((_mode == "relative") || (_mode == "rel"))
  {
    *_result = AxisPositionMode::kRelative;
    return true;
  }
  return false;
}

}  // namespace

class ZMotionDriverNode::Impl
{
  public:
  enum class PendingType
  {
    kVelocity = 0,
    kMoveAbsolute = 1,
    kMoveRelative = 2,
  };

  struct PendingCommand
  {
    PendingType type = PendingType::kVelocity;
    std::vector<int> logical_axes;
    std::vector<double> values;
  };

  explicit Impl(ZMotionDriverNode *_node)
      : node(_node),
        logger(_node->get_logger()),
        sdk(std::make_unique<ZMotionSdkWrapper>())
  {
  }

  ~Impl()
  {
    this->TearDown();
  }

  void WriteLogLocked(const std::string &_tag, const std::string &_message)
  {
    if (!this->log_stream.is_open())
    {
      return;
    }

    const auto stamp = this->node->now();
    this->log_stream << stamp.nanoseconds() << "," << SanitizeCsv(_tag) << ","
                     << SanitizeCsv(_message) << "\n";
    this->log_stream.flush();
  }

  bool LoadParameters()
  {
    // controller.* parameters
    this->controller_ip =
        this->node->get_parameter("controller.ip").as_string();

    this->ecat_config.slot_id =
        this->node->get_parameter("controller.slot_id").as_int();

    this->ecat_config.timeout_ms =
        this->node->get_parameter("controller.timeout_ms").as_int();

    this->feedback_period_ms =
        this->node->get_parameter("controller.feedback_period_ms").as_int();

    this->io_period_ms =
        this->node->get_parameter("controller.io_period_ms").as_int();

    this->min_remain_buffer =
        this->node->get_parameter("controller.min_remain_buffer").as_int();

    this->enable_axis_on_activate =
        this->node->get_parameter("controller.enable_axis_on_activate")
            .as_bool();

    this->log_file_path =
        this->node->get_parameter("controller.log_file").as_string();

    // control.* topics
    this->velocity_topic =
        this->node->get_parameter("control.velocity_topic").as_string();

    this->single_axis_topic =
        this->node->get_parameter("control.single_axis_topic").as_string();

    this->mimic_group_topics[0] =
        this->node->get_parameter("control.mimic_group1_topic").as_string();

    this->mimic_group_topics[1] =
        this->node->get_parameter("control.mimic_group2_topic").as_string();

    this->brake_topic =
        this->node->get_parameter("control.brake_topic").as_string();

    // recording parameters
    this->record_csv_enabled =
        this->node->get_parameter("feedback.record_csv_enabled").as_bool();

    this->record_csv_file =
        this->node->get_parameter("feedback.record_csv_file").as_string();

    this->record_rosbag_enabled =
        this->node->get_parameter("feedback.record_rosbag_enabled").as_bool();

    this->record_rosbag_file =
        this->node->get_parameter("feedback.record_rosbag_file").as_string();

    std::vector<int64_t> logical_indices;
    (void)this->node->get_parameter("axis.logical_indices", logical_indices);

    std::vector<std::string> joint_names;
    (void)this->node->get_parameter("axis.joint_names", joint_names);

    std::vector<std::string> control_modes;
    (void)this->node->get_parameter("axis.control_modes", control_modes);

    std::vector<std::string> position_modes;
    (void)this->node->get_parameter("axis.position_modes", position_modes);

    std::vector<double> zero_offsets;
    (void)this->node->get_parameter("axis.zero_offsets", zero_offsets);

    std::vector<double> units;
    (void)this->node->get_parameter("axis.units", units);

    std::vector<double> speeds;
    (void)this->node->get_parameter("axis.speeds", speeds);

    std::vector<double> accels;
    (void)this->node->get_parameter("axis.accels", accels);

    std::vector<double> decels;
    (void)this->node->get_parameter("axis.decels", decels);

    this->joint_state_topic =
        this->node->get_parameter("axis.joint_state_topic").as_string();

    std::vector<std::string> mimic_position_topics;
    (void)this->node->get_parameter("axis.mimic_position_topics",
                                    mimic_position_topics);
    if (mimic_position_topics.size() != 2U)
    {
      RCLCPP_ERROR(this->logger,
                   "axis.mimic_position_topics must contain exactly 2 topics");
      return false;
    }

    std::vector<std::string> position_topics;
    (void)this->node->get_parameter("axis.position_topics", position_topics);

    if ((logical_indices.size() != kAxisCount) ||
        (joint_names.size() != kAxisCount) ||
        (control_modes.size() != kAxisCount) ||
        (position_modes.size() != kAxisCount) ||
        (zero_offsets.size() != kAxisCount) || (units.size() != kAxisCount) ||
        (speeds.size() != kAxisCount) || (accels.size() != kAxisCount) ||
        (decels.size() != kAxisCount) || (position_topics.size() != kAxisCount))
    {
      RCLCPP_ERROR(this->logger, "axis.* arrays must all be length %zu",
                   kAxisCount);
      return false;
    }

    this->axes.clear();
    this->axes.reserve(kAxisCount);
    this->logical_to_axis_index.clear();

    std::set<int> logical_seen;

    for (std::size_t i = 0; i < kAxisCount; ++i)
    {
      AxisControlMode control_mode = AxisControlMode::kPosition;
      AxisPositionMode position_mode = AxisPositionMode::kAbsolute;
      if (!ParseControlMode(control_modes[i], &control_mode))
      {
        RCLCPP_ERROR(this->logger, "invalid axis.control_modes[%zu]: %s", i,
                     control_modes[i].c_str());
        return false;
      }
      if (!ParsePositionMode(position_modes[i], &position_mode))
      {
        RCLCPP_ERROR(this->logger, "invalid axis.position_modes[%zu]: %s", i,
                     position_modes[i].c_str());
        return false;
      }

      AxisConfig config;
      config.logical_index = static_cast<int>(logical_indices[i]);
      config.physical_axis = static_cast<int>(i);
      config.joint_name = joint_names[i];
      config.control_mode = control_mode;
      config.position_mode = position_mode;
      config.zero_offset = zero_offsets[i];
      config.units = units[i];
      config.speed = speeds[i];
      config.accel = accels[i];
      config.decel = decels[i];
      config.position_topic = position_topics[i];

      if (logical_seen.count(config.logical_index) > 0U)
      {
        RCLCPP_ERROR(this->logger, "duplicated logical axis index: %d",
                     config.logical_index);
        return false;
      }

      logical_seen.insert(config.logical_index);
      this->logical_to_axis_index[config.logical_index] = i;
      this->axes.push_back(config);
    }

    {
      std::vector<int64_t> vel_param;
      (void)this->node->get_parameter("control.velocity_logical_indices",
                                      vel_param);
      this->velocity_logical_axes = ConvertToIntVector(vel_param);
    }
    if (this->velocity_logical_axes.size() != 4U)
    {
      RCLCPP_ERROR(this->logger,
                   "control.velocity_logical_indices must contain 4 axes");
      return false;
    }
    for (std::size_t i = 0; i < this->velocity_logical_axes.size(); ++i)
    {
      auto it =
          this->logical_to_axis_index.find(this->velocity_logical_axes[i]);
      if (it == this->logical_to_axis_index.end())
      {
        RCLCPP_ERROR(this->logger,
                     "velocity axis not found in axis.logical_indices: %d",
                     this->velocity_logical_axes[i]);
        return false;
      }
      if (this->axes[it->second].control_mode != AxisControlMode::kVelocity)
      {
        RCLCPP_ERROR(this->logger, "velocity axis %d mode must be velocity",
                     this->velocity_logical_axes[i]);
        return false;
      }
    }

    this->single_logical_axis =
        this->node->get_parameter("control.single_axis_logical_index").as_int();
    if (this->logical_to_axis_index.count(this->single_logical_axis) == 0U)
    {
      RCLCPP_ERROR(this->logger,
                   "single axis not found in axis.logical_indices: %d",
                   this->single_logical_axis);
      return false;
    }

    {
      std::vector<int64_t> mg1;
      (void)this->node->get_parameter("control.mimic_group1_logical_indices",
                                      mg1);

      this->mimic_groups[0] = ConvertToIntVector(mg1);
    }
    {
      std::vector<int64_t> mg2;
      (void)this->node->get_parameter("control.mimic_group2_logical_indices",
                                      mg2);
      this->mimic_groups[1] = ConvertToIntVector(mg2);
    }

    this->mimic_member_logical_axes.clear();
    for (int group = 0; group < 2; ++group)
    {
      if (this->mimic_groups[group].size() != 2U)
      {
        RCLCPP_ERROR(this->logger, "mimic group %d must contain 2 logical axes",
                     group);
        return false;
      }

      AxisPositionMode expected_mode = AxisPositionMode::kAbsolute;
      bool expected_mode_set = false;
      for (std::size_t i = 0; i < this->mimic_groups[group].size(); ++i)
      {
        const int logical_axis = this->mimic_groups[group][i];
        auto it = this->logical_to_axis_index.find(logical_axis);
        if (it == this->logical_to_axis_index.end())
        {
          RCLCPP_ERROR(this->logger,
                       "mimic group axis %d not found in axis.logical_indices",
                       logical_axis);
          return false;
        }

        const AxisConfig &axis = this->axes[it->second];
        if (axis.control_mode != AxisControlMode::kPosition)
        {
          RCLCPP_ERROR(this->logger,
                       "mimic group axis %d mode must be position",
                       logical_axis);
          return false;
        }

        if (!expected_mode_set)
        {
          expected_mode = axis.position_mode;
          expected_mode_set = true;
        }
        else if (axis.position_mode != expected_mode)
        {
          RCLCPP_ERROR(this->logger,
                       "mimic group %d position modes must be identical",
                       group);
          return false;
        }

        this->mimic_member_logical_axes.insert(logical_axis);
      }
    }

    std::vector<int64_t> io_ids;
    (void)this->node->get_parameter("io.input_ids", io_ids);

    std::vector<std::string> io_topics;
    (void)this->node->get_parameter("io.state_topics", io_topics);

    std::vector<bool> io_estop;
    (void)this->node->get_parameter("io.emergency_stop_on_high", io_estop);

    // new: trigger modes per-IO (integer enum values)
    std::vector<int64_t> io_trigger_modes;

    (void)this->node->get_parameter("io.trigger_modes", io_trigger_modes);
    if (!io_ids.empty() && io_topics.empty())
    {
      io_topics.reserve(io_ids.size());
      for (std::size_t i = 0; i < io_ids.size(); ++i)
      {
        io_topics.push_back("io/input_" + std::to_string(io_ids[i]));
      }
    }

    if (io_topics.size() != io_ids.size())
    {
      RCLCPP_ERROR(this->logger,
                   "io.input_ids and io.state_topics size mismatch");
      return false;
    }

    if (io_estop.size() < io_ids.size())
    {
      io_estop.resize(io_ids.size(), false);
    }

    this->io_inputs.clear();
    this->io_inputs.reserve(io_ids.size());
    for (std::size_t i = 0; i < io_ids.size(); ++i)
    {
      IoInputConfig io;
      io.io_id = static_cast<int>(io_ids[i]);
      io.state_topic = io_topics[i];
      io.emergency_stop_on_high = io_estop[i];

      if (i < io_trigger_modes.size())
      {
        switch (static_cast<int>(io_trigger_modes[i]))
        {
          case 1:
            io.trigger_mode = IoTriggerMode::kLevelHigh;
            break;
          case 2:
            io.trigger_mode = IoTriggerMode::kLevelLow;
            break;
          case 3:
            io.trigger_mode = IoTriggerMode::kRisingEdge;
            break;
          case 4:
            io.trigger_mode = IoTriggerMode::kFallingEdge;
            break;
          case 5:
            io.trigger_mode = IoTriggerMode::kBothEdges;
            break;
          case 0:
          default:
            io.trigger_mode = IoTriggerMode::kNone;
            break;
        }
      }
      else
      {
        // compatibility: if emergency_stop_on_high was set, treat as level-high
        if (io.emergency_stop_on_high)
        {
          io.trigger_mode = IoTriggerMode::kLevelHigh;
        }
        else
        {
          io.trigger_mode = IoTriggerMode::kNone;
        }
      }

      this->io_inputs.push_back(io);
    }

    // initialize previous values cache for edge detection (-1 == unknown)
    this->previous_io_values.assign(this->io_inputs.size(), -1);

    this->ecat_config.init.InitStructFlag = 0;

    this->ecat_config.init.LocalAxisId =
        this->node->get_parameter("ecat.init.local_axis_id").as_int();

    this->ecat_config.init.LocalAxisNum =
        this->node->get_parameter("ecat.init.local_axis_num").as_int();

    this->ecat_config.init.DriveAxisStart =
        this->node->get_parameter("ecat.init.drive_axis_start").as_int();

    this->ecat_config.init.DriveAxisNum =
        this->node->get_parameter("ecat.init.drive_axis_num").as_int();

    this->ecat_config.init.DriveIoStara =
        this->node->get_parameter("ecat.init.drive_io_start").as_int();

    this->ecat_config.init.DriveIoSpa =
        this->node->get_parameter("ecat.init.drive_io_space").as_int();

    this->ecat_config.init.DriveEnable =
        this->node->get_parameter("ecat.init.drive_enable").as_int();

    this->ecat_config.init.EcatNodeNum =
        this->node->get_parameter("ecat.init.ecat_node_num").as_int();

    this->ecat_config.init.SysClockMode =
        this->node->get_parameter("ecat.init.sys_clock_mode").as_int();

    this->ecat_config.init.BusRedSwitch =
        this->node->get_parameter("ecat.init.bus_red_switch").as_int();

    this->ecat_config.init.RedSpareSlot =
        this->node->get_parameter("ecat.init.red_spare_slot").as_int();

    std::vector<int64_t> drive_pdo_modes;
    (void)this->node->get_parameter("ecat.init.drive_pdo_mode",
                                    drive_pdo_modes);

    std::vector<int64_t> node_io_ids;
    (void)this->node->get_parameter("ecat.init.node_io_id", node_io_ids);

    std::vector<int64_t> node_aio_ids;
    (void)this->node->get_parameter("ecat.init.node_aio_id", node_aio_ids);

    std::vector<int64_t> dc_offset_flags;
    (void)this->node->get_parameter("ecat.init.dc_offset_flag",
                                    dc_offset_flags);

    std::vector<double> dc_offset_times;
    (void)this->node->get_parameter("ecat.init.dc_offset_time",
                                    dc_offset_times);

    for (int i = 0; i < 128; ++i)
    {
      this->ecat_config.init.DrivePdoMode[i] = 0;
      this->ecat_config.init.NodeIoId[i] = 0;
      this->ecat_config.init.NodeAIoId[i] = 0;
      this->ecat_config.init.DcOffsetFlag[i] = 0;
      this->ecat_config.init.DcOffsetTime[i] = 0.0f;
    }

    if (drive_pdo_modes.empty())
    {
      const int axis_num = std::max(0, this->ecat_config.init.DriveAxisNum);
      for (int i = 0; (i < axis_num) && (i < 128); ++i)
      {
        this->ecat_config.init.DrivePdoMode[i] = -1;
      }
    }
    else
    {
      for (std::size_t i = 0; (i < drive_pdo_modes.size()) && (i < 128U); ++i)
      {
        this->ecat_config.init.DrivePdoMode[i] =
            static_cast<int>(drive_pdo_modes[i]);
      }
    }

    for (std::size_t i = 0; (i < node_io_ids.size()) && (i < 128U); ++i)
    {
      this->ecat_config.init.NodeIoId[i] = static_cast<int>(node_io_ids[i]);
    }
    for (std::size_t i = 0; (i < node_aio_ids.size()) && (i < 128U); ++i)
    {
      this->ecat_config.init.NodeAIoId[i] = static_cast<int>(node_aio_ids[i]);
    }
    for (std::size_t i = 0; (i < dc_offset_flags.size()) && (i < 128U); ++i)
    {
      this->ecat_config.init.DcOffsetFlag[i] =
          static_cast<int>(dc_offset_flags[i]);
    }
    for (std::size_t i = 0; (i < dc_offset_times.size()) && (i < 128U); ++i)
    {
      this->ecat_config.init.DcOffsetTime[i] =
          static_cast<float>(dc_offset_times[i]);
    }

    // Initialize last-known feedback containers (NaN = unknown)
    this->last_known_positions.assign(this->axes.size(),
                                      std::numeric_limits<double>::quiet_NaN());
    this->last_known_velocities.assign(
        this->axes.size(), std::numeric_limits<double>::quiet_NaN());
    this->last_known_efforts.assign(this->axes.size(),
                                    std::numeric_limits<double>::quiet_NaN());

    return true;
  }

  // 这个函数每次执行，都是在文件末尾续写，打开的是CSV文件
  bool OpenLogFile()
  {
    this->log_stream.open(this->log_file_path, std::ios::out | std::ios::app);
    if (!this->log_stream.is_open())
    {
      RCLCPP_ERROR(this->logger, "failed to open log file: %s",
                   this->log_file_path.c_str());
      return false;
    }

    if (this->log_stream.tellp() == std::streampos(0))
    {
      this->log_stream << "stamp_ns,tag,message\n";
      this->log_stream.flush();
    }
    return true;
  }

  // 关闭csv文件
  void CloseLogFile()
  {
    if (this->log_stream.is_open())
    {
      this->log_stream.flush();
      this->log_stream.close();
    }
  }

  bool OpenJointStateFile()
  {
    if (!this->record_csv_enabled)
    {
      return true;
    }

    this->joint_state_stream.open(this->record_csv_file,
                                  std::ios::out | std::ios::app);
    if (!this->joint_state_stream.is_open())
    {
      RCLCPP_ERROR(this->logger, "failed to open joint_state file: %s",
                   this->record_csv_file.c_str());
      return false;
    }

    if (this->joint_state_stream.tellp() == std::streampos(0))
    {
      this->joint_state_stream
          << "stamp_ns,names,positions,velocities,efforts\n";
      this->joint_state_stream.flush();
    }
    return true;
  }

  void CloseJointStateFile()
  {
    std::lock_guard<std::mutex> lock(this->joint_state_stream_mutex);
    if (this->joint_state_stream.is_open())
    {
      this->joint_state_stream.flush();
      this->joint_state_stream.close();
    }
  }

  void WriteJointStateCsv(const sensor_msgs::msg::JointState &_js)
  {
    if (!this->record_csv_enabled)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(this->joint_state_stream_mutex);
    if (!this->joint_state_stream.is_open())
    {
      return;
    }

    const auto stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    const std::string names = JoinStrings(_js.name);
    const std::string positions = JoinDoubles(_js.position);
    const std::string velocities = JoinDoubles(_js.velocity);
    const std::string efforts = JoinDoubles(_js.effort);

    this->joint_state_stream << stamp.nanoseconds() << "," << SanitizeCsv(names)
                             << "," << SanitizeCsv(positions) << ","
                             << SanitizeCsv(velocities) << ","
                             << SanitizeCsv(efforts) << "\n";
    this->joint_state_stream.flush();
  }

  bool StartRosbagRecorder()
  {
    if (!this->record_rosbag_enabled)
    {
      return true;
    }
    if (this->rosbag_pid > 0)
    {
      // already running
      return true;
    }

    pid_t pid = fork();
    if (pid < 0)
    {
      RCLCPP_ERROR(this->logger, "failed to fork for rosbag recorder");
      return false;
    }
    if (pid == 0)
    {
      // child: exec ros2 bag record -o <file> <topic>
      execlp("ros2", "ros2", "bag", "record", "-o",
             this->record_rosbag_file.c_str(), this->joint_state_topic.c_str(),
             (char *)NULL);
      _exit(127);
    }
    this->rosbag_pid = pid;
    RCLCPP_INFO(this->logger, "started rosbag record pid=%d",
                static_cast<int>(pid));
    return true;
  }

  void StopRosbagRecorder()
  {
    if (this->rosbag_pid <= 0)
    {
      return;
    }
    (void)kill(this->rosbag_pid, SIGINT);
    int status = 0;
    (void)waitpid(this->rosbag_pid, &status, 0);
    RCLCPP_INFO(this->logger, "stopped rosbag record pid=%d",
                static_cast<int>(this->rosbag_pid));
    this->rosbag_pid = -1;
  }

  bool ConfigureHardware()
  {
    std::lock_guard<std::mutex> lock(this->mutex);

    this->emergency_stop = false;
    CallResult connect_result = this->sdk->Connect(this->controller_ip);
    if (!connect_result.ok)
    {
      RCLCPP_ERROR(this->logger, "connect failed: %s",
                   connect_result.message.c_str());
      return false;
    }

    CallResult ecat_result = this->sdk->InitEthercat(this->ecat_config);
    if (!ecat_result.ok)
    {
      RCLCPP_ERROR(this->logger, "InitEthercat failed: %s",
                   ecat_result.message.c_str());
      (void)this->sdk->Disconnect();
      return false;
    }

    for (std::size_t i = 0; i < this->axes.size(); ++i)
    {
      const AxisConfig &axis = this->axes[i];
      CallResult axis_result = this->sdk->ConfigureAxis(
          axis.physical_axis, axis.units, axis.speed, axis.accel, axis.decel);
      if (!axis_result.ok)
      {
        RCLCPP_ERROR(this->logger,
                     "ConfigureAxis failed (logical=%d physical=%d): %s",
                     axis.logical_index, axis.physical_axis,
                     axis_result.message.c_str());
        (void)this->sdk->Disconnect();
        return false;
      }

      (void)this->sdk->SetAxisEnable(axis.physical_axis, false);
    }

    this->WriteLogLocked("lifecycle", "configure_hardware_ok");
    return true;
  }

  bool CreateInterfaces()
  {
    this->joint_state_pub =
        this->node->create_publisher<sensor_msgs::msg::JointState>(
            this->joint_state_topic, rclcpp::SystemDefaultsQoS());

    this->axis_position_pubs.assign(this->axes.size(), nullptr);
    for (std::size_t i = 0; i < this->axes.size(); ++i)
    {
      if (this->mimic_member_logical_axes.count(this->axes[i].logical_index) >
          0U)
      {
        continue;
      }
      this->axis_position_pubs[i] =
          this->node->create_publisher<std_msgs::msg::Float64>(
              this->axes[i].position_topic, rclcpp::SystemDefaultsQoS());
    }

    this->mimic_position_pubs[0] =
        this->node->create_publisher<std_msgs::msg::Float64>(
            this->mimic_position_topics[0], rclcpp::SystemDefaultsQoS());
    this->mimic_position_pubs[1] =
        this->node->create_publisher<std_msgs::msg::Float64>(
            this->mimic_position_topics[1], rclcpp::SystemDefaultsQoS());

    this->io_state_pubs.clear();
    for (std::size_t i = 0; i < this->io_inputs.size(); ++i)
    {
      this->io_state_pubs.push_back(
          this->node->create_publisher<std_msgs::msg::Bool>(
              this->io_inputs[i].state_topic, rclcpp::SystemDefaultsQoS()));
    }

    this->velocity_sub =
        this->node->create_subscription<std_msgs::msg::Float64MultiArray>(
            this->velocity_topic, rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr _msg)
            { this->OnVelocityCommand(_msg); });

    this->single_axis_sub =
        this->node->create_subscription<std_msgs::msg::Float64>(
            this->single_axis_topic, rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Float64::SharedPtr _msg)
            { this->OnSingleAxisCommand(_msg); });

    this->mimic_group_subs[0] =
        this->node->create_subscription<std_msgs::msg::Float64>(
            this->mimic_group_topics[0], rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Float64::SharedPtr _msg)
            { this->OnMimicCommand(0, _msg); });

    this->mimic_group_subs[1] =
        this->node->create_subscription<std_msgs::msg::Float64>(
            this->mimic_group_topics[1], rclcpp::SystemDefaultsQoS(),
            [this](const std_msgs::msg::Float64::SharedPtr _msg)
            { this->OnMimicCommand(1, _msg); });

    this->brake_sub = this->node->create_subscription<std_msgs::msg::Bool>(
        this->brake_topic, rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Bool::SharedPtr _msg)
        { this->OnBrakeCommand(_msg); });

    this->feedback_timer = this->node->create_wall_timer(
        std::chrono::milliseconds(this->feedback_period_ms),
        [this]() { this->PublishFeedback(); });

    this->io_timer = this->node->create_wall_timer(
        std::chrono::milliseconds(this->io_period_ms),
        [this]() { this->PollIoInputs(); });

    return true;
  }

  void ResetInterfaces()
  {
    this->feedback_timer.reset();
    this->io_timer.reset();

    this->velocity_sub.reset();
    this->single_axis_sub.reset();
    this->mimic_group_subs[0].reset();
    this->mimic_group_subs[1].reset();
    this->brake_sub.reset();

    this->joint_state_pub.reset();
    this->axis_position_pubs.clear();
    this->mimic_position_pubs[0].reset();
    this->mimic_position_pubs[1].reset();
    this->io_state_pubs.clear();
  }

  void TearDown()
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->active = false;
      this->configured = false;
      this->emergency_stop = false;
      this->previous_io_values.clear();
      if (this->sdk)
      {
        (void)this->sdk->StopAll();
        for (std::size_t i = 0; i < this->axes.size(); ++i)
        {
          (void)this->sdk->SetAxisEnable(this->axes[i].physical_axis, false);
        }
        (void)this->sdk->Disconnect();
      }
      this->WriteLogLocked("lifecycle", "teardown");
    }

    this->ResetInterfaces();
    this->CloseLogFile();
    this->StopRosbagRecorder();
    this->CloseJointStateFile();
  }

  std::string DescribeCommand(const PendingCommand &_cmd) const
  {
    std::ostringstream oss;
    switch (_cmd.type)
    {
      case PendingType::kVelocity:
        oss << "velocity";
        break;
      case PendingType::kMoveAbsolute:
        oss << "move_abs";
        break;
      case PendingType::kMoveRelative:
        oss << "move_rel";
        break;
      default:
        oss << "unknown";
        break;
    }
    oss << " axes=" << JoinInts(_cmd.logical_axes)
        << " values=" << JoinDoubles(_cmd.values);
    return oss.str();
  }

  void DispatchCommand(const PendingCommand &_cmd)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->DispatchCommandLocked(_cmd);
  }

  void DispatchCommandLocked(const PendingCommand &_cmd)
  {
    if (!this->configured || !this->active || this->emergency_stop)
    {
      return;
    }

    this->WriteLogLocked("control_rx", this->DescribeCommand(_cmd));

    std::vector<int> physical_axes;
    if (!this->MapLogicalAxesToPhysical(_cmd.logical_axes, &physical_axes))
    {
      this->WriteLogLocked("control_err", "logical axis map failed");
      RCLCPP_ERROR(this->logger, "logical axis map failed: %s",
                   this->DescribeCommand(_cmd).c_str());
      return;
    }

    if ((_cmd.type != PendingType::kVelocity) &&
        (_cmd.type != PendingType::kMoveAbsolute) &&
        (_cmd.type != PendingType::kMoveRelative))
    {
      this->WriteLogLocked("control_err", "unknown command type");
      RCLCPP_ERROR(this->logger, "unknown command type: %s",
                   this->DescribeCommand(_cmd).c_str());
      return;
    }

    if (_cmd.values.size() != physical_axes.size())
    {
      this->WriteLogLocked("control_err", "command value size mismatch");
      RCLCPP_ERROR(this->logger, "command value size mismatch: %s",
                   this->DescribeCommand(_cmd).c_str());
      return;
    }

    CallResult buffer_result = this->CheckHardwareBufferLocked(physical_axes);
    if (!buffer_result.ok)
    {
      this->WriteLogLocked("control_err", buffer_result.message);
      RCLCPP_ERROR(this->logger, "hardware remain buffer check failed: %s",
                   buffer_result.message.c_str());
      this->TriggerEmergencyStopLocked(buffer_result.message);
      return;
    }

    CallResult exec_result = this->ExecuteCommandLocked(_cmd, physical_axes);
    if (!exec_result.ok)
    {
      this->WriteLogLocked("control_err", exec_result.message);
      RCLCPP_ERROR(this->logger, "execute command failed: %s",
                   exec_result.message.c_str());
      this->TriggerEmergencyStopLocked(exec_result.message);
      return;
    }

    this->WriteLogLocked("control_tx", this->DescribeCommand(_cmd));
  }

  bool MapLogicalAxesToPhysical(const std::vector<int> &_logical_axes,
                                std::vector<int> *_physical_axes) const
  {
    if (_physical_axes == nullptr)
    {
      return false;
    }

    _physical_axes->clear();
    _physical_axes->reserve(_logical_axes.size());
    for (std::size_t i = 0; i < _logical_axes.size(); ++i)
    {
      auto it = this->logical_to_axis_index.find(_logical_axes[i]);
      if (it == this->logical_to_axis_index.end())
      {
        return false;
      }
      _physical_axes->push_back(this->axes[it->second].physical_axis);
    }
    return true;
  }

  CallResult CheckHardwareBufferLocked(
      const std::vector<int> &_physical_axes) const
  {
    for (std::size_t i = 0; i < _physical_axes.size(); ++i)
    {
      int remain = 0;
      CallResult remain_result =
          this->sdk->GetRemainBuffer(_physical_axes[i], &remain);
      if (!remain_result.ok)
      {
        return remain_result;
      }

      if (remain <= this->min_remain_buffer)
      {
        return CallResult::Failure(
            -202, "hardware remain buffer is below threshold", false);
      }
    }

    return CallResult::Success();
  }

  CallResult ExecuteCommandLocked(const PendingCommand &_cmd,
                                  const std::vector<int> &_physical_axes)
  {
    if ((_cmd.type == PendingType::kVelocity) &&
        (_cmd.values.size() == _physical_axes.size()))
    {
      for (std::size_t i = 0; i < _physical_axes.size(); ++i)
      {
        CallResult result =
            this->sdk->CommandVelocity(_physical_axes[i], _cmd.values[i]);
        if (!result.ok)
        {
          return result;
        }
      }
      return CallResult::Success();
    }

    if (_cmd.values.size() != _physical_axes.size())
    {
      return CallResult::Failure(-204, "command value size mismatch", false);
    }

    std::vector<double> axis_values;
    axis_values.reserve(_cmd.values.size());
    for (std::size_t i = 0; i < _cmd.values.size(); ++i)
    {
      const int logical_axis = _cmd.logical_axes[i];
      auto it = this->logical_to_axis_index.find(logical_axis);
      if (it == this->logical_to_axis_index.end())
      {
        return CallResult::Failure(-205, "logical axis not found", false);
      }

      const AxisConfig &axis = this->axes[it->second];
      if (_cmd.type == PendingType::kMoveAbsolute)
      {
        axis_values.push_back(_cmd.values[i] + axis.zero_offset);
      }
      else
      {
        axis_values.push_back(_cmd.values[i]);
      }
    }

    if (_cmd.type == PendingType::kMoveAbsolute)
    {
      return this->sdk->MoveAbsoluteMulti(_physical_axes, axis_values);
    }
    if (_cmd.type == PendingType::kMoveRelative)
    {
      return this->sdk->MoveRelativeMulti(_physical_axes, axis_values);
    }

    return CallResult::Failure(-206, "unknown command type", false);
  }

  void TriggerEmergencyStopLocked(const std::string &_reason)
  {
    const bool was_emergency_stop = this->emergency_stop;
    this->emergency_stop = true;

    (void)this->sdk->StopAll();
    for (std::size_t i = 0; i < this->axes.size(); ++i)
    {
      (void)this->sdk->SetAxisEnable(this->axes[i].physical_axis, false);
    }

    if (was_emergency_stop)
    {
      return;
    }

    this->WriteLogLocked("safety", "emergency_stop:" + _reason);
    RCLCPP_ERROR(this->logger, "emergency stop triggered: %s", _reason.c_str());
  }
  void PublishFeedback()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->configured || !this->active ||
        (this->joint_state_pub == nullptr))
    {
      return;
    }
    if (!this->joint_state_pub->is_activated())
    {
      return;
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->node->now();
    joint_state.name.reserve(this->axes.size());
    joint_state.position.reserve(this->axes.size());
    joint_state.velocity.reserve(this->axes.size());
    joint_state.effort.reserve(this->axes.size());

    std::vector<double> logical_positions(this->axes.size(), 0.0);
    std::vector<double> logical_velocities(this->axes.size(), 0.0);

    for (std::size_t i = 0; i < this->axes.size(); ++i)
    {
      const AxisConfig &axis = this->axes[i];

      double mpos = 0.0;
      double mspeed = 0.0;
      CallResult mpos_result = this->sdk->GetMpos(axis.physical_axis, &mpos);
      CallResult speed_result =
          this->sdk->GetMspeed(axis.physical_axis, &mspeed);

      double logical_position = std::numeric_limits<double>::quiet_NaN();
      double logical_velocity = std::numeric_limits<double>::quiet_NaN();
      double effort = std::numeric_limits<double>::quiet_NaN();

      if (mpos_result.ok && speed_result.ok)
      {
        logical_position = mpos - axis.zero_offset;
        logical_velocity = mspeed;
        // update last-known values
        if (i < this->last_known_positions.size())
        {
          this->last_known_positions[i] = logical_position;
        }
        if (i < this->last_known_velocities.size())
        {
          this->last_known_velocities[i] = logical_velocity;
        }
        // effort not read here; preserve last_known_efforts if any
        if (i < this->last_known_efforts.size())
        {
          effort = this->last_known_efforts[i];
        }
      }
      else
      {
        // read failed: log and fall back to last-known or NaN
        this->WriteLogLocked(
            "feedback_err",
            "read failed axis=" + std::to_string(axis.logical_index));
        if (i < this->last_known_positions.size() &&
            !std::isnan(this->last_known_positions[i]))
        {
          logical_position = this->last_known_positions[i];
        }
        if (i < this->last_known_velocities.size() &&
            !std::isnan(this->last_known_velocities[i]))
        {
          logical_velocity = this->last_known_velocities[i];
        }
        if (i < this->last_known_efforts.size() &&
            !std::isnan(this->last_known_efforts[i]))
        {
          effort = this->last_known_efforts[i];
        }
      }

      logical_positions[i] = logical_position;
      logical_velocities[i] = logical_velocity;

      joint_state.name.push_back(axis.joint_name);
      joint_state.position.push_back(logical_position);
      joint_state.velocity.push_back(logical_velocity);
      joint_state.effort.push_back(effort);

      if ((i < this->axis_position_pubs.size()) &&
          (this->axis_position_pubs[i] != nullptr) &&
          this->axis_position_pubs[i]->is_activated())
      {
        if (!std::isnan(logical_position))
        {
          std_msgs::msg::Float64 position_msg;
          position_msg.data = logical_position;
          this->axis_position_pubs[i]->publish(position_msg);
        }
      }
    }

    this->joint_state_pub->publish(joint_state);

    // record joint_state to CSV if enabled
    this->WriteJointStateCsv(joint_state);

    for (int group = 0; group < 2; ++group)
    {
      if ((this->mimic_position_pubs[group] == nullptr) ||
          !this->mimic_position_pubs[group]->is_activated() ||
          this->mimic_groups[group].empty())
      {
        continue;
      }

      const int logical_axis = this->mimic_groups[group][0];
      auto it = this->logical_to_axis_index.find(logical_axis);
      if (it == this->logical_to_axis_index.end())
      {
        continue;
      }

      std_msgs::msg::Float64 position_msg;
      position_msg.data = logical_positions[it->second];
      this->mimic_position_pubs[group]->publish(position_msg);
    }

    this->WriteLogLocked(
        "feedback",
        "joint_state_size=" + std::to_string(joint_state.name.size()) +
            " positions=" + JoinDoubles(logical_positions) +
            " velocities=" + JoinDoubles(logical_velocities));
  }

  void PollIoInputs()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->configured)
    {
      return;
    }

    std::vector<int> io_values;
    io_values.reserve(this->io_inputs.size());

    for (std::size_t i = 0; i < this->io_inputs.size(); ++i)
    {
      int value = 0;
      CallResult result = this->sdk->GetInput(this->io_inputs[i].io_id, &value);
      if (!result.ok)
      {
        this->WriteLogLocked(
            "io_err",
            "read io failed id=" + std::to_string(this->io_inputs[i].io_id));
        io_values.push_back(value);
        continue;
      }

      // publish state and dispatch configured trigger action (synchronous,
      // locked)
      this->DispatchIoTriggerLocked(i, value);

      // update previous value for edge detection
      if (i < this->previous_io_values.size())
      {
        this->previous_io_values[i] = value;
      }

      io_values.push_back(value);
    }

    if (!io_values.empty())
    {
      this->WriteLogLocked("io", "io_values=" + JoinInts(io_values));
    }
  }

  void DispatchIoTriggerLocked(std::size_t index, int current_value)
  {
    if (index >= this->io_inputs.size())
    {
      return;
    }

    const IoInputConfig &cfg = this->io_inputs[index];

    // Always publish state to the state topic if available
    if ((index < this->io_state_pubs.size()) &&
        (this->io_state_pubs[index] != nullptr) &&
        this->io_state_pubs[index]->is_activated())
    {
      std_msgs::msg::Bool msg;
      msg.data = (current_value != 0);
      this->io_state_pubs[index]->publish(msg);
    }

    int prev = -1;
    if (index < this->previous_io_values.size())
    {
      prev = this->previous_io_values[index];
    }
    const bool prev_known = (prev != -1);
    const bool changed = prev_known && (prev != current_value);
    const bool rising = prev_known && (prev == 0 && current_value != 0);
    const bool falling = prev_known && (prev != 0 && current_value == 0);
    const bool high = (current_value != 0);
    const bool low = !high;

    this->WriteLogLocked(
        "io_trigger", "io_id=" + std::to_string(cfg.io_id) + " mode=" +
                          std::to_string(static_cast<int>(cfg.trigger_mode)) +
                          " prev=" + std::to_string(prev) +
                          " cur=" + std::to_string(current_value));

    // Maintain compatibility: explicit emergency_stop_on_high still forces an
    // emergency stop
    if (cfg.emergency_stop_on_high && high && !this->emergency_stop)
    {
      this->TriggerEmergencyStopLocked("io_" + std::to_string(cfg.io_id));
      return;
    }

    bool should_trigger = false;
    switch (cfg.trigger_mode)
    {
      case IoTriggerMode::kNone:
        should_trigger = false;
        break;
      case IoTriggerMode::kLevelHigh:
        should_trigger = high;
        break;
      case IoTriggerMode::kLevelLow:
        should_trigger = low;
        break;
      case IoTriggerMode::kRisingEdge:
        should_trigger = rising;
        break;
      case IoTriggerMode::kFallingEdge:
        should_trigger = falling;
        break;
      case IoTriggerMode::kBothEdges:
        should_trigger = changed;
        break;
      default:
        should_trigger = false;
        break;
    }

    if (!should_trigger)
    {
      return;
    }

    // Execute per-IO action synchronously while mutex is held.
    // Keep actions minimal to avoid blocking the polling loop.
    // IO pairs: (0,1)->logical 0, (2,3)->logical 1, (4,5)->logical 2,
    // (6,7)->logical 3
    if ((cfg.io_id >= 0) && (cfg.io_id <= 7))
    {
      if (!this->configured || !this->active || this->emergency_stop)
      {
        this->WriteLogLocked(
            "io_action",
            "io_" + std::to_string(cfg.io_id) +
                " triggered but node not active/configured or in emergency");
        return;
      }

      // Map IO pair to logical axis: integer division by 2 gives 0..3
      const int logical_axis = cfg.io_id / 2;

      // Determine a sensible velocity magnitude: prefer configured axis speed
      double speed_mag = 0.1;
      auto it = this->logical_to_axis_index.find(logical_axis);
      if (it != this->logical_to_axis_index.end())
      {
        speed_mag = this->axes[it->second].speed;
      }

      double target_vel = 0.0;
      // even IO id -> positive, odd -> negative
      if ((cfg.io_id % 2) == 0)
      {
        target_vel = (high ? speed_mag : 0.0);
      }
      else
      {
        target_vel = (high ? -speed_mag : 0.0);
      }

      this->WriteLogLocked("io_action", "io_" + std::to_string(cfg.io_id) +
                                            " triggered: set velocity=" +
                                            std::to_string(target_vel));

      PendingCommand cmd;
      cmd.type = PendingType::kVelocity;
      cmd.logical_axes = std::vector<int>{logical_axis};
      cmd.values = std::vector<double>{target_vel};
      this->DispatchCommandLocked(cmd);
    }
    else
    {
      this->WriteLogLocked("io_action",
                           "io_" + std::to_string(cfg.io_id) + " triggered");
    }
  }

  void OnVelocityCommand(
      const std_msgs::msg::Float64MultiArray::SharedPtr &_msg)
  {
    if (_msg == nullptr)
    {
      return;
    }

    if (_msg->data.size() != this->velocity_logical_axes.size())
    {
      RCLCPP_ERROR(this->logger,
                   "velocity command size mismatch: expected=%zu got=%zu",
                   this->velocity_logical_axes.size(), _msg->data.size());
      return;
    }

    PendingCommand cmd;
    cmd.type = PendingType::kVelocity;
    cmd.logical_axes = this->velocity_logical_axes;
    cmd.values.assign(_msg->data.begin(), _msg->data.end());
    this->DispatchCommand(cmd);
  }

  void OnSingleAxisCommand(const std_msgs::msg::Float64::SharedPtr &_msg)
  {
    if (_msg == nullptr)
    {
      return;
    }

    auto it = this->logical_to_axis_index.find(this->single_logical_axis);
    if (it == this->logical_to_axis_index.end())
    {
      return;
    }

    const AxisConfig &axis = this->axes[it->second];
    PendingCommand cmd;
    cmd.logical_axes = std::vector<int>{this->single_logical_axis};
    cmd.values = std::vector<double>{_msg->data};

    if (axis.control_mode == AxisControlMode::kVelocity)
    {
      cmd.type = PendingType::kVelocity;
    }
    else if (axis.position_mode == AxisPositionMode::kAbsolute)
    {
      cmd.type = PendingType::kMoveAbsolute;
    }
    else
    {
      cmd.type = PendingType::kMoveRelative;
    }

    this->DispatchCommand(cmd);
  }

  void OnMimicCommand(const int _group,
                      const std_msgs::msg::Float64::SharedPtr &_msg)
  {
    if ((_msg == nullptr) || (_group < 0) || (_group > 1))
    {
      return;
    }

    if (this->mimic_groups[_group].size() != 2U)
    {
      return;
    }

    const int logical_axis = this->mimic_groups[_group][0];
    auto it = this->logical_to_axis_index.find(logical_axis);
    if (it == this->logical_to_axis_index.end())
    {
      return;
    }

    const AxisConfig &axis = this->axes[it->second];

    PendingCommand cmd;
    cmd.logical_axes = this->mimic_groups[_group];
    cmd.values = std::vector<double>{_msg->data, _msg->data};

    if (axis.position_mode == AxisPositionMode::kAbsolute)
    {
      cmd.type = PendingType::kMoveAbsolute;
    }
    else
    {
      cmd.type = PendingType::kMoveRelative;
    }

    this->DispatchCommand(cmd);
  }

  void OnBrakeCommand(const std_msgs::msg::Bool::SharedPtr &_msg)
  {
    if (_msg == nullptr)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(this->mutex);

    if (_msg->data)
    {
      this->TriggerEmergencyStopLocked("brake_topic");
      return;
    }

    if (!this->emergency_stop)
    {
      return;
    }

    this->WriteLogLocked("safety", "brake_release_requires_lifecycle_recovery");
    RCLCPP_WARN(this->logger,
                "brake released but emergency stop remains latched; lifecycle "
                "recovery required");
  }

  bool ActivateNode()
  {
    std::lock_guard<std::mutex> lock(this->mutex);

    if (this->enable_axis_on_activate && !this->emergency_stop)
    {
      for (std::size_t i = 0; i < this->axes.size(); ++i)
      {
        CallResult result =
            this->sdk->SetAxisEnable(this->axes[i].physical_axis, true);
        if (!result.ok)
        {
          RCLCPP_ERROR(this->logger, "axis enable failed (logical=%d): %s",
                       this->axes[i].logical_index, result.message.c_str());

          (void)this->sdk->StopAll();
          for (std::size_t j = 0; j < this->axes.size(); ++j)
          {
            (void)this->sdk->SetAxisEnable(this->axes[j].physical_axis, false);
          }
          this->active = false;
          this->WriteLogLocked("lifecycle", "activate_failed");
          return false;
        }
      }
    }

    this->active = true;

    if (this->joint_state_pub != nullptr)
    {
      this->joint_state_pub->on_activate();
    }

    for (std::size_t i = 0; i < this->axis_position_pubs.size(); ++i)
    {
      if (this->axis_position_pubs[i] != nullptr)
      {
        this->axis_position_pubs[i]->on_activate();
      }
    }

    for (int group = 0; group < 2; ++group)
    {
      if (this->mimic_position_pubs[group] != nullptr)
      {
        this->mimic_position_pubs[group]->on_activate();
      }
    }

    for (std::size_t i = 0; i < this->io_state_pubs.size(); ++i)
    {
      if (this->io_state_pubs[i] != nullptr)
      {
        this->io_state_pubs[i]->on_activate();
      }
    }

    // start rosbag recorder if requested
    if (!this->StartRosbagRecorder())
    {
      RCLCPP_ERROR(this->logger, "failed to start rosbag recorder");
    }

    this->WriteLogLocked("lifecycle", "activate");
    return true;
  }

  bool DeactivateNode()
  {
    std::lock_guard<std::mutex> lock(this->mutex);

    this->active = false;
    this->emergency_stop = false;
    this->previous_io_values.clear();

    (void)this->sdk->StopAll();
    for (std::size_t i = 0; i < this->axes.size(); ++i)
    {
      (void)this->sdk->SetAxisEnable(this->axes[i].physical_axis, false);
    }

    if (this->joint_state_pub != nullptr)
    {
      this->joint_state_pub->on_deactivate();
    }
    for (std::size_t i = 0; i < this->axis_position_pubs.size(); ++i)
    {
      if (this->axis_position_pubs[i] != nullptr)
      {
        this->axis_position_pubs[i]->on_deactivate();
      }
    }
    for (int group = 0; group < 2; ++group)
    {
      if (this->mimic_position_pubs[group] != nullptr)
      {
        this->mimic_position_pubs[group]->on_deactivate();
      }
    }
    for (std::size_t i = 0; i < this->io_state_pubs.size(); ++i)
    {
      if (this->io_state_pubs[i] != nullptr)
      {
        this->io_state_pubs[i]->on_deactivate();
      }
    }

    // stop rosbag recorder if running
    this->StopRosbagRecorder();

    this->WriteLogLocked("lifecycle", "deactivate");
    return true;
  }

  bool CleanupNode()
  {
    this->TearDown();
    return true;
  }

  bool configured = false;
  bool active = false;
  bool emergency_stop = false;

  ZMotionDriverNode *node;
  rclcpp::Logger logger;

  std::unique_ptr<ZMotionSdkWrapper> sdk;

  std::mutex mutex;

  std::string controller_ip;
  std::string log_file_path;

  EcatConfig ecat_config;

  int feedback_period_ms = 20;
  int io_period_ms = 20;
  int min_remain_buffer = 20;
  bool enable_axis_on_activate = true;

  std::string velocity_topic;
  std::string single_axis_topic;
  std::string mimic_group_topics[2];
  std::string brake_topic;
  std::string joint_state_topic;
  std::string mimic_position_topics[2];

  std::vector<AxisConfig> axes;
  std::unordered_map<int, std::size_t> logical_to_axis_index;
  std::vector<int> velocity_logical_axes;
  int single_logical_axis = -1;
  std::vector<int> mimic_groups[2];
  std::set<int> mimic_member_logical_axes;

  std::vector<IoInputConfig> io_inputs;
  std::vector<int> previous_io_values;

  // Last-known feedback values (persist across PublishFeedback calls).
  std::vector<double> last_known_positions;
  std::vector<double> last_known_velocities;
  std::vector<double> last_known_efforts;

  std::ofstream log_stream;

  std::ofstream joint_state_stream;
  std::mutex joint_state_stream_mutex;
  bool record_csv_enabled = false;
  std::string record_csv_file;
  bool record_rosbag_enabled = false;
  std::string record_rosbag_file;
  pid_t rosbag_pid = -1;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_pub;
  std::vector<
      rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr>
      axis_position_pubs;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr
      mimic_position_pubs[2];
  std::vector<
      rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr>
      io_state_pubs;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      velocity_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr single_axis_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr mimic_group_subs[2];
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_sub;

  rclcpp::TimerBase::SharedPtr feedback_timer;
  rclcpp::TimerBase::SharedPtr io_timer;
};

ZMotionDriverNode::ZMotionDriverNode(const rclcpp::NodeOptions &_options)
    : rclcpp_lifecycle::LifecycleNode("zmotion_driver", _options),
      pimpl_(std::make_unique<Impl>(this))
{
}

ZMotionDriverNode::~ZMotionDriverNode() = default;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZMotionDriverNode::on_configure(const rclcpp_lifecycle::State &_state)
{
  (void)_state;

  if (!this->pimpl_->LoadParameters())
  {
    return CallbackReturn::FAILURE;
  }

  if (!this->pimpl_->OpenLogFile())
  {
    return CallbackReturn::FAILURE;
  }

  if (!this->pimpl_->OpenJointStateFile())
  {
    this->pimpl_->TearDown();
    return CallbackReturn::FAILURE;
  }

  if (!this->pimpl_->ConfigureHardware())
  {
    this->pimpl_->TearDown();
    return CallbackReturn::FAILURE;
  }

  if (!this->pimpl_->CreateInterfaces())
  {
    this->pimpl_->TearDown();
    return CallbackReturn::FAILURE;
  }

  this->pimpl_->configured = true;
  RCLCPP_INFO(this->get_logger(), "configure successfully.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZMotionDriverNode::on_activate(const rclcpp_lifecycle::State &_state)
{
  (void)_state;
  if (!this->pimpl_->ActivateNode())
  {
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(this->get_logger(), "activate successfully.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZMotionDriverNode::on_deactivate(const rclcpp_lifecycle::State &_state)
{
  (void)_state;
  if (!this->pimpl_->DeactivateNode())
  {
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(this->get_logger(), "deactivate successfully.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZMotionDriverNode::on_cleanup(const rclcpp_lifecycle::State &_state)
{
  (void)_state;
  if (!this->pimpl_->CleanupNode())
  {
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(this->get_logger(), "cleanup successfully.");
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ZMotionDriverNode::on_shutdown(const rclcpp_lifecycle::State &_state)
{
  (void)_state;
  this->pimpl_->TearDown();
  RCLCPP_INFO(this->get_logger(), "shutdown successfully.");
  return CallbackReturn::SUCCESS;
}

}  // namespace zmotion_driver
