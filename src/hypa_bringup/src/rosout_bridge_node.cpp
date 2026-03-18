// rosout_bridge 节点 - 订阅 /rosout 并将日志实时写入文件
#include <rmw/qos_profiles.h>

#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace fs = std::filesystem;

/// ROS 2 日志桥接节点
class RosoutBridgeNode : public rclcpp_lifecycle::LifecycleNode
{
  public:
  explicit RosoutBridgeNode(
      const rclcpp::NodeOptions &_options = rclcpp::NodeOptions())
      : rclcpp_lifecycle::LifecycleNode("rosout_bridge", _options)
  {
    // 声明参数
    this->declare_parameter<std::string>("log_file_path", "~/.ros/hypa_logs");
    this->declare_parameter<int>("log_publish_frequency", 100);
    this->declare_parameter<int>(
        "min_log_level", static_cast<int>(rcl_interfaces::msg::Log::DEBUG));
  }

  ~RosoutBridgeNode() override
  {
    if (this->log_file_.is_open())
    {
      this->log_file_.close();
    }
  }

  protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &_state) override
  {
    (void)_state;

    RCLCPP_INFO(this->get_logger(), "[rosout_bridge] 配置生命周期节点");

    try
    {
      // 获取参数
      auto log_file_path = this->get_parameter("log_file_path").as_string();
      auto min_log_level =
          static_cast<uint8_t>(this->get_parameter("min_log_level").as_int());

      // 展开家目录
      if (log_file_path.front() == '~')
      {
        const char *home = std::getenv("HOME");
        if (home)
        {
          log_file_path = std::string(home) + log_file_path.substr(1);
        }
      }

      // 创建日志目录
      fs::create_directories(log_file_path);

      // 生成日期格式的文件名：YYYY-MM-DD.log
      auto now = std::time(nullptr);
      auto tm = std::localtime(&now);
      std::ostringstream oss;
      oss << std::put_time(tm, "%Y-%m-%d");

      this->log_file_path_ = log_file_path;
      this->current_date_ = oss.str();
      this->min_log_level_ = min_log_level;

      // 打开日志文件（追加模式）
      std::string filename =
          fs::path(log_file_path) / (this->current_date_ + ".log");
      this->log_file_.open(filename, std::ios::app);

      if (!this->log_file_.is_open())
      {
        RCLCPP_ERROR(this->get_logger(), "[rosout_bridge] 无法打开日志文件: %s",
                     filename.c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::ERROR;
      }

      RCLCPP_INFO(this->get_logger(), "[rosout_bridge] 日志文件已打开: %s",
                  filename.c_str());

      // 订阅 /rosout
      rclcpp::QoS log_qos(
          rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
      log_qos.keep_last(100);
      this->subscription_ = this->create_subscription<rcl_interfaces::msg::Log>(
          "/rosout", log_qos,
          [this](const rcl_interfaces::msg::Log::SharedPtr _msg)
          { this->onLogMessage(_msg); });

      RCLCPP_INFO(this->get_logger(), "[rosout_bridge] 已订阅 /rosout 话题");

      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::SUCCESS;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "[rosout_bridge] 配置失败: %s",
                   e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::ERROR;
    }
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &_state) override
  {
    (void)_state;
    RCLCPP_INFO(this->get_logger(), "[rosout_bridge] 激活节点");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &_state) override
  {
    (void)_state;
    RCLCPP_INFO(this->get_logger(), "[rosout_bridge] 停用节点");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &_state) override
  {
    (void)_state;
    RCLCPP_INFO(this->get_logger(), "[rosout_bridge] 清理节点");
    if (this->log_file_.is_open())
    {
      this->log_file_.close();
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  private:
  void onLogMessage(const rcl_interfaces::msg::Log::SharedPtr _msg)
  {
    if (!_msg || static_cast<uint8_t>(_msg->level) < this->min_log_level_)
      return;

    // 检查日期是否改变（用于按天分文件）
    auto now = std::time(nullptr);
    auto tm = std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(tm, "%Y-%m-%d");
    std::string today = oss.str();

    if (today != this->current_date_)
    {
      // 关闭当前文件，打开新文件
      if (this->log_file_.is_open())
      {
        this->log_file_.close();
      }

      this->current_date_ = today;
      std::string filename =
          fs::path(this->log_file_path_) / (this->current_date_ + ".log");
      this->log_file_.open(filename, std::ios::app);

      RCLCPP_INFO(this->get_logger(), "[rosout_bridge] 切换到新日志文件: %s",
                  filename.c_str());
    }

    // 格式化日志消息
    std::ostringstream log_stream;
    log_stream << "[" << std::put_time(tm, "%H:%M:%S") << "] ["
               << levelToString(_msg->level) << "] [" << _msg->name
               << "]: " << _msg->msg;

    std::string log_line = log_stream.str();

    // 写入文件
    if (this->log_file_.is_open())
    {
      this->log_file_ << log_line << "\n";
      this->log_file_.flush();
    }

    // 可选：同时打印到终端
    // RCLCPP_DEBUG(this->get_logger(), "%s", log_line.c_str());
  }

  static std::string levelToString(uint8_t _level)
  {
    switch (_level)
    {
      case rcl_interfaces::msg::Log::DEBUG:
        return "DEBUG";
      case rcl_interfaces::msg::Log::INFO:
        return "INFO";
      case rcl_interfaces::msg::Log::WARN:
        return "WARN";
      case rcl_interfaces::msg::Log::ERROR:
        return "ERROR";
      case rcl_interfaces::msg::Log::FATAL:
        return "FATAL";
      default:
        return "UNKNOWN";
    }
  }

  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
  std::ofstream log_file_;
  std::string log_file_path_;
  std::string current_date_;
  uint8_t min_log_level_ = rcl_interfaces::msg::Log::DEBUG;
};

int main(int _argc, char *_argv[])
{
  rclcpp::init(_argc, _argv);
  auto node = std::make_shared<RosoutBridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
