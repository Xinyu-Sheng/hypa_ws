#include "zmotion_driver/zmotion_driver_node.hpp"

#include <algorithm>
#include <chrono>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
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

std::vector<int64_t> DefaultIntSequence(const int _count)
{
  std::vector<int64_t> values;
  values.reserve(static_cast<std::size_t>(_count));
  for (int i = 0; i < _count; ++i)
  {
    values.push_back(i);
  }
  return values;
}

std::vector<std::string> DefaultNames(const std::string &_prefix,
                                      const int _count)
{
  std::vector<std::string> names;
  names.reserve(static_cast<std::size_t>(_count));
  for (int i = 0; i < _count; ++i)
  {
    names.push_back(_prefix + std::to_string(i));
  }
  return names;
}

std::vector<double> DefaultDoubleValues(const int _count, const double _value)
{
  return std::vector<double>(static_cast<std::size_t>(_count), _value);
}

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

  std::string ResolveTopic(const std::string &_topic) const
  {
    if (_topic.empty())
    {
      return _topic;
    }

    if (_topic.front() == '/')
    {
      return _topic;
    }

    if (this->namespace_param.empty())
    {
      return _topic;
    }

    std::string ns = this->namespace_param;
    if (ns.front() != '/')
    {
      ns = "/" + ns;
    }
    while ((!ns.empty()) && (ns.back() == '/'))
    {
      ns.pop_back();
    }
    return ns + "/" + _topic;
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
    if (!this->node->has_parameter("namespace"))
    {
      this->node->declare_parameter<std::string>("namespace", "");
    }
    if (!this->node->has_parameter("use_sim_time"))
    {
      this->node->declare_parameter<bool>("use_sim_time", false);
    }
    if (!this->node->has_parameter("robot_name"))
    {
      this->node->declare_parameter<std::string>("robot_name", "hypa");
    }

    this->namespace_param = this->node->get_parameter("namespace").as_string();
    this->robot_name = this->node->get_parameter("robot_name").as_string();

    this->controller_ip = this->node->declare_parameter<std::string>(
        "controller.ip", "192.168.0.11");
    this->ecat_config.slot_id =
        this->node->declare_parameter<int>("controller.slot_id", 0);
    this->ecat_config.backup_slot_id =
        this->node->declare_parameter<int>("controller.backup_slot_id", 1);
    this->ecat_config.timeout_ms =
        this->node->declare_parameter<int>("controller.timeout_ms", 10000);

    this->control_period_ms =
        this->node->declare_parameter<int>("controller.control_period_ms", 5);
    this->feedback_period_ms =
        this->node->declare_parameter<int>("controller.feedback_period_ms", 20);
    this->io_period_ms =
        this->node->declare_parameter<int>("controller.io_period_ms", 20);

    this->min_remain_buffer =
        this->node->declare_parameter<int>("controller.min_remain_buffer", 20);
    this->queue_size =
        static_cast<std::size_t>(this->node->declare_parameter<int>(
            "controller.command_queue_size", 4096));
    this->enable_axis_on_activate = this->node->declare_parameter<bool>(
        "controller.enable_axis_on_activate", true);
    this->disable_buffer_check_on_error = this->node->declare_parameter<bool>(
        "controller.disable_buffer_check_on_error", true);
    this->log_file_path = this->node->declare_parameter<std::string>(
        "controller.log_file", "/tmp/zmotion_driver_log.csv");

    this->velocity_topic =
        this->ResolveTopic(this->node->declare_parameter<std::string>(
            "control.velocity_topic", "cmd/velocity_axes"));
    this->single_axis_topic =
        this->ResolveTopic(this->node->declare_parameter<std::string>(
            "control.single_axis_topic", "cmd/single_axis"));
    this->mimic_group_topics[0] =
        this->ResolveTopic(this->node->declare_parameter<std::string>(
            "control.mimic_group1_topic", "cmd/mimic_group1"));
    this->mimic_group_topics[1] =
        this->ResolveTopic(this->node->declare_parameter<std::string>(
            "control.mimic_group2_topic", "cmd/mimic_group2"));
    this->brake_topic =
        this->ResolveTopic(this->node->declare_parameter<std::string>(
            "control.brake_topic", "cmd/brake"));
    this->joint_state_topic =
        this->ResolveTopic(this->node->declare_parameter<std::string>(
            "feedback.joint_state_topic", "joint_states"));

    const std::vector<std::string> mimic_position_topics =
        this->node->declare_parameter<std::vector<std::string>>(
            "feedback.mimic_position_topics",
            std::vector<std::string>{"feedback/mimic_group1/position",
                                     "feedback/mimic_group2/position"});
    if (mimic_position_topics.size() != 2U)
    {
      RCLCPP_ERROR(
          this->logger,
          "feedback.mimic_position_topics must contain exactly 2 topics");
      return false;
    }
    this->mimic_position_topics[0] =
        this->ResolveTopic(mimic_position_topics[0]);
    this->mimic_position_topics[1] =
        this->ResolveTopic(mimic_position_topics[1]);

    const std::vector<int64_t> logical_indices =
        this->node->declare_parameter<std::vector<int64_t>>(
            "axis.logical_indices",
            DefaultIntSequence(static_cast<int>(kAxisCount)));
    const std::vector<int64_t> physical_axes =
        this->node->declare_parameter<std::vector<int64_t>>(
            "axis.physical_axes",
            DefaultIntSequence(static_cast<int>(kAxisCount)));
    const std::vector<std::string> logical_names =
        this->node->declare_parameter<std::vector<std::string>>(
            "axis.logical_names",
            DefaultNames("logical_axis_", static_cast<int>(kAxisCount)));
    const std::vector<std::string> joint_names =
        this->node->declare_parameter<std::vector<std::string>>(
            "axis.joint_names",
            DefaultNames("joint_", static_cast<int>(kAxisCount)));
    const std::vector<std::string> control_modes =
        this->node->declare_parameter<std::vector<std::string>>(
            "axis.control_modes",
            std::vector<std::string>{"velocity", "velocity", "velocity",
                                     "velocity", "position", "position",
                                     "position", "position", "position"});
    const std::vector<std::string> position_modes =
        this->node->declare_parameter<std::vector<std::string>>(
            "axis.position_modes",
            std::vector<std::string>{"absolute", "absolute", "absolute",
                                     "absolute", "absolute", "absolute",
                                     "absolute", "absolute", "absolute"});
    const std::vector<double> zero_offsets =
        this->node->declare_parameter<std::vector<double>>(
            "axis.zero_offsets",
            DefaultDoubleValues(static_cast<int>(kAxisCount), 0.0));
    const std::vector<double> units =
        this->node->declare_parameter<std::vector<double>>(
            "axis.units",
            DefaultDoubleValues(static_cast<int>(kAxisCount), 1.0));
    const std::vector<double> speeds =
        this->node->declare_parameter<std::vector<double>>(
            "axis.speeds",
            DefaultDoubleValues(static_cast<int>(kAxisCount), 10.0));
    const std::vector<double> accels =
        this->node->declare_parameter<std::vector<double>>(
            "axis.accels",
            DefaultDoubleValues(static_cast<int>(kAxisCount), 100.0));
    const std::vector<double> decels =
        this->node->declare_parameter<std::vector<double>>(
            "axis.decels",
            DefaultDoubleValues(static_cast<int>(kAxisCount), 100.0));
    const std::vector<std::string> position_topics =
        this->node->declare_parameter<std::vector<std::string>>(
            "axis.position_topics",
            std::vector<std::string>{
                "feedback/axis0/position", "feedback/axis1/position",
                "feedback/axis2/position", "feedback/axis3/position",
                "feedback/axis4/position", "feedback/axis5/position",
                "feedback/axis6/position", "feedback/axis7/position",
                "feedback/axis8/position"});

    if ((logical_indices.size() != kAxisCount) ||
        (physical_axes.size() != kAxisCount) ||
        (logical_names.size() != kAxisCount) ||
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
    std::set<int> physical_seen;

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
      config.physical_axis = static_cast<int>(physical_axes[i]);
      config.logical_name = logical_names[i];
      config.joint_name = joint_names[i];
      config.control_mode = control_mode;
      config.position_mode = position_mode;
      config.zero_offset = zero_offsets[i];
      config.units = units[i];
      config.speed = speeds[i];
      config.accel = accels[i];
      config.decel = decels[i];
      config.position_topic = this->ResolveTopic(position_topics[i]);

      if (logical_seen.count(config.logical_index) > 0U)
      {
        RCLCPP_ERROR(this->logger, "duplicated logical axis index: %d",
                     config.logical_index);
        return false;
      }
      if (physical_seen.count(config.physical_axis) > 0U)
      {
        RCLCPP_ERROR(this->logger, "duplicated physical axis index: %d",
                     config.physical_axis);
        return false;
      }

      logical_seen.insert(config.logical_index);
      physical_seen.insert(config.physical_axis);
      this->logical_to_axis_index[config.logical_index] = i;
      this->axes.push_back(config);
    }

    this->velocity_logical_axes =
        ConvertToIntVector(this->node->declare_parameter<std::vector<int64_t>>(
            "control.velocity_logical_indices",
            std::vector<int64_t>{0, 1, 2, 3}));
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

    this->single_logical_axis = this->node->declare_parameter<int>(
        "control.single_axis_logical_index", 4);
    if (this->logical_to_axis_index.count(this->single_logical_axis) == 0U)
    {
      RCLCPP_ERROR(this->logger,
                   "single axis not found in axis.logical_indices: %d",
                   this->single_logical_axis);
      return false;
    }

    this->mimic_groups[0] =
        ConvertToIntVector(this->node->declare_parameter<std::vector<int64_t>>(
            "control.mimic_group1_logical_indices",
            std::vector<int64_t>{5, 6}));
    this->mimic_groups[1] =
        ConvertToIntVector(this->node->declare_parameter<std::vector<int64_t>>(
            "control.mimic_group2_logical_indices",
            std::vector<int64_t>{7, 8}));

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

    const std::vector<int64_t> io_ids =
        this->node->declare_parameter<std::vector<int64_t>>(
            "io.input_ids", std::vector<int64_t>{});
    std::vector<std::string> io_topics =
        this->node->declare_parameter<std::vector<std::string>>(
            "io.state_topics", std::vector<std::string>{});
    std::vector<bool> io_estop =
        this->node->declare_parameter<std::vector<bool>>(
            "io.emergency_stop_on_high", std::vector<bool>{});

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
      io.state_topic = this->ResolveTopic(io_topics[i]);
      io.emergency_stop_on_high = io_estop[i];
      this->io_inputs.push_back(io);
    }

    this->ecat_config.init.InitStructFlag = 0;
    this->ecat_config.init.LocalAxisId =
        this->node->declare_parameter<int>("ecat.init.local_axis_id", 0);
    this->ecat_config.init.LocalAxisNum =
        this->node->declare_parameter<int>("ecat.init.local_axis_num", 0);
    this->ecat_config.init.DriveAxisStart =
        this->node->declare_parameter<int>("ecat.init.drive_axis_start", 0);
    this->ecat_config.init.DriveAxisNum =
        this->node->declare_parameter<int>("ecat.init.drive_axis_num", 9);
    this->ecat_config.init.DriveIoStara =
        this->node->declare_parameter<int>("ecat.init.drive_io_start", 256);
    this->ecat_config.init.DriveIoSpa =
        this->node->declare_parameter<int>("ecat.init.drive_io_space", 16);
    this->ecat_config.init.DriveEnable =
        this->node->declare_parameter<int>("ecat.init.drive_enable", 0);
    this->ecat_config.init.EcatNodeNum =
        this->node->declare_parameter<int>("ecat.init.ecat_node_num", -1);
    this->ecat_config.init.SysClockMode =
        this->node->declare_parameter<int>("ecat.init.sys_clock_mode", 1);
    this->ecat_config.init.BusRedSwitch =
        this->node->declare_parameter<int>("ecat.init.bus_red_switch", 0);
    this->ecat_config.init.RedSpareSlot =
        this->node->declare_parameter<int>("ecat.init.red_spare_slot", 0);

    const std::vector<int64_t> drive_pdo_modes =
        this->node->declare_parameter<std::vector<int64_t>>(
            "ecat.init.drive_pdo_mode", std::vector<int64_t>{});
    const std::vector<int64_t> node_io_ids =
        this->node->declare_parameter<std::vector<int64_t>>(
            "ecat.init.node_io_id", std::vector<int64_t>{});
    const std::vector<int64_t> node_aio_ids =
        this->node->declare_parameter<std::vector<int64_t>>(
            "ecat.init.node_aio_id", std::vector<int64_t>{});
    const std::vector<int64_t> dc_offset_flags =
        this->node->declare_parameter<std::vector<int64_t>>(
            "ecat.init.dc_offset_flag", std::vector<int64_t>{});
    const std::vector<double> dc_offset_times =
        this->node->declare_parameter<std::vector<double>>(
            "ecat.init.dc_offset_time", std::vector<double>{});

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
        this->ecat_config.init.DrivePdoMode[i] = 10;
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

    return true;
  }

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

  void CloseLogFile()
  {
    if (this->log_stream.is_open())
    {
      this->log_stream.flush();
      this->log_stream.close();
    }
  }

  bool ConfigureHardware()
  {
    std::lock_guard<std::mutex> lock(this->mutex);

    this->remain_buffer_check_enabled = true;
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

    this->control_timer = this->node->create_wall_timer(
        std::chrono::milliseconds(this->control_period_ms),
        [this]() { this->ProcessPendingCommands(); });

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
    this->control_timer.reset();
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
      this->pending_commands.clear();
      this->emergency_stop = false;
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

  void EnqueueCommand(const PendingCommand &_cmd)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->configured)
    {
      return;
    }

    if (this->pending_commands.size() >= this->queue_size)
    {
      this->pending_commands.pop_front();
      RCLCPP_WARN(this->logger, "command queue overflow, drop oldest command");
    }
    this->pending_commands.push_back(_cmd);
    this->WriteLogLocked("control_rx", this->DescribeCommand(_cmd));
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

  CallResult CheckHardwareBufferLocked(const PendingCommand &_cmd)
  {
    if (!this->remain_buffer_check_enabled)
    {
      return CallResult::Success();
    }

    if (this->min_remain_buffer < 0)
    {
      return CallResult::Success();
    }

    std::vector<int> physical_axes;
    if (!this->MapLogicalAxesToPhysical(_cmd.logical_axes, &physical_axes))
    {
      return CallResult::Failure(-201, "logical axis map failed", false);
    }

    for (std::size_t i = 0; i < physical_axes.size(); ++i)
    {
      int remain = 0;
      CallResult remain_result =
          this->sdk->GetRemainBuffer(physical_axes[i], &remain);
      if (!remain_result.ok)
      {
        if (this->disable_buffer_check_on_error)
        {
          this->remain_buffer_check_enabled = false;
          RCLCPP_WARN(this->logger,
                      "disable hardware remain buffer check due to error: %s",
                      remain_result.message.c_str());
          return CallResult::Success();
        }
        return remain_result;
      }

      if (remain <= this->min_remain_buffer)
      {
        return CallResult::Failure(
            -202, "hardware remain buffer is below threshold", true);
      }
    }

    return CallResult::Success();
  }

  CallResult ExecuteCommandLocked(const PendingCommand &_cmd)
  {
    std::vector<int> physical_axes;
    if (!this->MapLogicalAxesToPhysical(_cmd.logical_axes, &physical_axes))
    {
      return CallResult::Failure(-203, "logical axis map failed", false);
    }

    if ((_cmd.type == PendingType::kVelocity) &&
        (_cmd.values.size() == physical_axes.size()))
    {
      for (std::size_t i = 0; i < physical_axes.size(); ++i)
      {
        CallResult result =
            this->sdk->CommandVelocity(physical_axes[i], _cmd.values[i]);
        if (!result.ok)
        {
          return result;
        }
      }
      return CallResult::Success();
    }

    if (_cmd.values.size() != physical_axes.size())
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
      return this->sdk->MoveAbsoluteMulti(physical_axes, axis_values);
    }
    if (_cmd.type == PendingType::kMoveRelative)
    {
      return this->sdk->MoveRelativeMulti(physical_axes, axis_values);
    }

    return CallResult::Failure(-206, "unknown command type", false);
  }

  void TriggerEmergencyStopLocked(const std::string &_reason)
  {
    this->pending_commands.clear();
    this->emergency_stop = true;

    (void)this->sdk->StopAll();
    for (std::size_t i = 0; i < this->axes.size(); ++i)
    {
      (void)this->sdk->SetAxisEnable(this->axes[i].physical_axis, false);
    }

    this->WriteLogLocked("safety", "emergency_stop:" + _reason);
    RCLCPP_ERROR(this->logger, "emergency stop triggered: %s", _reason.c_str());
  }

  void ProcessPendingCommands()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->configured || !this->active || this->emergency_stop)
    {
      return;
    }

    std::size_t processed = 0U;
    while ((!this->pending_commands.empty()) && (processed < 128U))
    {
      const PendingCommand command = this->pending_commands.front();
      CallResult buffer_result = this->CheckHardwareBufferLocked(command);
      if (!buffer_result.ok)
      {
        if (!buffer_result.retriable)
        {
          RCLCPP_ERROR(this->logger, "buffer check failed: %s",
                       buffer_result.message.c_str());
          this->WriteLogLocked("control_err", buffer_result.message);
          this->pending_commands.pop_front();
          continue;
        }
        break;
      }

      CallResult exec_result = this->ExecuteCommandLocked(command);
      if (exec_result.ok)
      {
        this->WriteLogLocked("control_tx", this->DescribeCommand(command));
        this->pending_commands.pop_front();
        ++processed;
        continue;
      }

      if (exec_result.retriable)
      {
        this->WriteLogLocked("control_retry", exec_result.message);
        break;
      }

      this->WriteLogLocked("control_err", exec_result.message);
      RCLCPP_ERROR(this->logger, "execute command failed: %s",
                   exec_result.message.c_str());
      this->pending_commands.pop_front();
    }
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

    for (std::size_t i = 0; i < this->axes.size(); ++i)
    {
      const AxisConfig &axis = this->axes[i];

      double mpos = 0.0;
      double mspeed = 0.0;
      CallResult mpos_result = this->sdk->GetMpos(axis.physical_axis, &mpos);
      CallResult speed_result =
          this->sdk->GetMspeed(axis.physical_axis, &mspeed);

      if (!mpos_result.ok || !speed_result.ok)
      {
        this->WriteLogLocked(
            "feedback_err",
            "read failed axis=" + std::to_string(axis.logical_index));
        continue;
      }

      const double logical_position = mpos - axis.zero_offset;
      logical_positions[i] = logical_position;

      joint_state.name.push_back(axis.joint_name);
      joint_state.position.push_back(logical_position);
      joint_state.velocity.push_back(mspeed);
      joint_state.effort.push_back(0.0);

      if ((i < this->axis_position_pubs.size()) &&
          (this->axis_position_pubs[i] != nullptr) &&
          this->axis_position_pubs[i]->is_activated())
      {
        std_msgs::msg::Float64 position_msg;
        position_msg.data = logical_position;
        this->axis_position_pubs[i]->publish(position_msg);
      }
    }

    this->joint_state_pub->publish(joint_state);

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
        "joint_state_size=" + std::to_string(joint_state.name.size()));
  }

  void PollIoInputs()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->configured)
    {
      return;
    }

    for (std::size_t i = 0; i < this->io_inputs.size(); ++i)
    {
      int value = 0;
      CallResult result = this->sdk->GetInput(this->io_inputs[i].io_id, &value);
      if (!result.ok)
      {
        this->WriteLogLocked(
            "io_err",
            "read io failed id=" + std::to_string(this->io_inputs[i].io_id));
        continue;
      }

      if ((i < this->io_state_pubs.size()) &&
          (this->io_state_pubs[i] != nullptr) &&
          this->io_state_pubs[i]->is_activated())
      {
        std_msgs::msg::Bool msg;
        msg.data = (value != 0);
        this->io_state_pubs[i]->publish(msg);
      }

      if (this->io_inputs[i].emergency_stop_on_high && (value != 0) &&
          !this->emergency_stop)
      {
        this->TriggerEmergencyStopLocked(
            "io_" + std::to_string(this->io_inputs[i].io_id));
      }
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
    this->EnqueueCommand(cmd);
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

    this->EnqueueCommand(cmd);
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

    this->EnqueueCommand(cmd);
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

    this->emergency_stop = false;
    if (this->active && this->enable_axis_on_activate)
    {
      for (std::size_t i = 0; i < this->axes.size(); ++i)
      {
        (void)this->sdk->SetAxisEnable(this->axes[i].physical_axis, true);
      }
    }
    this->WriteLogLocked("safety", "brake_release");
  }

  bool ActivateNode()
  {
    std::lock_guard<std::mutex> lock(this->mutex);

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
          return false;
        }
      }
    }

    this->WriteLogLocked("lifecycle", "activate");
    return true;
  }

  bool DeactivateNode()
  {
    std::lock_guard<std::mutex> lock(this->mutex);

    this->active = false;
    this->pending_commands.clear();

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
  bool remain_buffer_check_enabled = true;

  ZMotionDriverNode *node;
  rclcpp::Logger logger;

  std::unique_ptr<ZMotionSdkWrapper> sdk;

  std::mutex mutex;

  std::string namespace_param;
  std::string robot_name;
  std::string controller_ip;
  std::string log_file_path;

  EcatConfig ecat_config;

  int control_period_ms = 5;
  int feedback_period_ms = 20;
  int io_period_ms = 20;
  int min_remain_buffer = 20;
  std::size_t queue_size = 4096;
  bool enable_axis_on_activate = true;
  bool disable_buffer_check_on_error = true;

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

  std::deque<PendingCommand> pending_commands;

  std::ofstream log_stream;

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

  rclcpp::TimerBase::SharedPtr control_timer;
  rclcpp::TimerBase::SharedPtr feedback_timer;
  rclcpp::TimerBase::SharedPtr io_timer;
};

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

ZMotionDriverNode::ZMotionDriverNode(const rclcpp::NodeOptions &_options)
    : rclcpp_lifecycle::LifecycleNode("zmotion_driver", _options),
      pimpl_(std::make_unique<Impl>(this))
{
}

ZMotionDriverNode::~ZMotionDriverNode() = default;

CallbackReturn ZMotionDriverNode::on_configure(
    const rclcpp_lifecycle::State &_state)
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

  if (!this->pimpl_->ConfigureHardware())
  {
    this->pimpl_->CloseLogFile();
    return CallbackReturn::FAILURE;
  }

  if (!this->pimpl_->CreateInterfaces())
  {
    this->pimpl_->TearDown();
    return CallbackReturn::FAILURE;
  }

  this->pimpl_->configured = true;
  RCLCPP_INFO(this->get_logger(), "configured with robot_name=%s namespace=%s",
              this->pimpl_->robot_name.c_str(),
              this->pimpl_->namespace_param.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZMotionDriverNode::on_activate(
    const rclcpp_lifecycle::State &_state)
{
  (void)_state;
  if (!this->pimpl_->ActivateNode())
  {
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZMotionDriverNode::on_deactivate(
    const rclcpp_lifecycle::State &_state)
{
  (void)_state;
  if (!this->pimpl_->DeactivateNode())
  {
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZMotionDriverNode::on_cleanup(
    const rclcpp_lifecycle::State &_state)
{
  (void)_state;
  if (!this->pimpl_->CleanupNode())
  {
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZMotionDriverNode::on_shutdown(
    const rclcpp_lifecycle::State &_state)
{
  (void)_state;
  this->pimpl_->TearDown();
  return CallbackReturn::SUCCESS;
}

}  // namespace zmotion_driver
