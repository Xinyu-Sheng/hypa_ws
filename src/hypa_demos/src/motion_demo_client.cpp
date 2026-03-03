#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "hypa_msgs/msg/motion_command.hpp"
#include "hypa_msgs/msg/motion_status.hpp"

using namespace std::chrono_literals;

namespace hypa_demos
{

class MotionDemoClient : public rclcpp::Node
{
 public:
  explicit MotionDemoClient(const rclcpp::NodeOptions& _options)
      : Node("motion_demo_client", _options)
  {
    // 声明参数
    this->declare_parameter("namespace", "");
    this->declare_parameter("robot_name", "hypa");
    this->declare_parameter("motion_command_topic", "motion_command");
    this->declare_parameter("motion_status_topic", "motion_status");

    // 获取参数
    auto namespace_val = this->get_parameter("namespace").as_string();
    auto robot_name = this->get_parameter("robot_name").as_string();
    command_topic_ = this->get_parameter("motion_command_topic").as_string();
    status_topic_ = this->get_parameter("motion_status_topic").as_string();

    // 构建带命名空间的主题名
    if (!namespace_val.empty())
    {
      command_topic_ =
          "/" + namespace_val + "/" + robot_name + "/" + command_topic_;
      status_topic_ =
          "/" + namespace_val + "/" + robot_name + "/" + status_topic_;
    }
    else if (!robot_name.empty() && robot_name != "hypa")
    {
      command_topic_ = "/" + robot_name + "/" + command_topic_;
      status_topic_ = "/" + robot_name + "/" + status_topic_;
    }

    // 创建发布器和订阅器
    command_publisher_ = this->create_publisher<hypa_msgs::msg::MotionCommand>(
        command_topic_, 10);
    status_subscription_ =
        this->create_subscription<hypa_msgs::msg::MotionStatus>(
            status_topic_, 10,
            std::bind(&MotionDemoClient::StatusCallback, this,
                      std::placeholders::_1));

    // 等待一小段时间确保订阅器就绪
    rclcpp::sleep_for(500ms);

    this->timer_ = this->create_wall_timer(
        1s, std::bind(&MotionDemoClient::RunDemo, this));

    RCLCPP_INFO(this->get_logger(), "Motion Demo Client initialized.");
    RCLCPP_INFO(this->get_logger(), "Command topic: %s",
                command_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Status topic: %s", status_topic_.c_str());
  }

 private:
  rclcpp::Publisher<hypa_msgs::msg::MotionCommand>::SharedPtr
      command_publisher_;
  rclcpp::Subscription<hypa_msgs::msg::MotionStatus>::SharedPtr
      status_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  int demo_step_ = 0;
  bool command_sent_ = false;
  bool motion_complete_ = false;
  std::string command_topic_;
  std::string status_topic_;

  void RunDemo()
  {
    if (this->timer_)
    {
      this->timer_->cancel();
    }

    switch (this->demo_step_)
    {
      case 0:
        this->TestSingleAxisMotion();
        break;
      case 1:
        this->TestInterpolatedMotion();
        break;
      case 2:
        this->TestCancelMotion();
        break;
      default:
        RCLCPP_INFO(this->get_logger(), "All demo steps completed.");
        rclcpp::shutdown();
        break;
    }
  }

  void TestSingleAxisMotion()
  {
    RCLCPP_INFO(this->get_logger(),
                "--- Step 1: Single Axis Motion (Axis 0 to 100.0) ---");

    auto cmd = hypa_msgs::msg::MotionCommand();
    cmd.axes = {0};
    cmd.positions = {100.0};
    cmd.velocities = {20.0};
    cmd.motion_type = 1;  // SINGLE_AXIS
    cmd.accelerations = {100.0};
    cmd.decelerations = {100.0};
    cmd.wait_time = 0.0;

    this->SendCommand(cmd);
    this->demo_step_ = 1;
  }

  void TestInterpolatedMotion()
  {
    RCLCPP_INFO(
        this->get_logger(),
        "--- Step 2: Dual Axis Interpolation (Axis 0,1 to 50.0, 50.0) ---");

    auto cmd = hypa_msgs::msg::MotionCommand();
    cmd.axes = {0, 1};
    cmd.positions = {50.0, 50.0};
    cmd.velocities = {10.0, 10.0};
    cmd.motion_type = 2;         // INTERPOLATED
    cmd.interpolation_mode = 0;  // LINEAR
    cmd.accelerations = {50.0, 50.0};
    cmd.decelerations = {50.0, 50.0};
    cmd.wait_time = 0.0;

    this->SendCommand(cmd);
    this->demo_step_ = 2;
  }

  void TestCancelMotion()
  {
    RCLCPP_INFO(
        this->get_logger(),
        "--- Step 3: Cancellation Demo (Long move, cancel after 2s) ---");

    auto cmd = hypa_msgs::msg::MotionCommand();
    cmd.axes = {0};
    cmd.positions = {500.0};
    cmd.velocities = {5.0};  // Slow move
    cmd.motion_type = 1;
    cmd.accelerations = {10.0};
    cmd.decelerations = {10.0};
    cmd.wait_time = 0.0;

    this->SendCommand(cmd);

    // 2秒后发送停止命令
    this->timer_ = this->create_wall_timer(
        2s,
        [this]()
        {
          RCLCPP_INFO(this->get_logger(), "Sending stop command...");
          auto stop_cmd = hypa_msgs::msg::MotionCommand();
          stop_cmd.axes = {0};
          stop_cmd.positions = {0.0};  // 目标位置不重要，用于触发停止
          stop_cmd.velocities = {0.0};
          stop_cmd.motion_type = 1;
          this->command_publisher_->publish(stop_cmd);
        });

    this->demo_step_ = 3;
  }

  void SendCommand(const hypa_msgs::msg::MotionCommand& _cmd)
  {
    RCLCPP_INFO(this->get_logger(), "Publishing motion command...");
    command_publisher_->publish(_cmd);
    command_sent_ = true;
    motion_complete_ = false;

    // 设置超时定时器，如果5秒内没有完成，则继续下一步
    this->timer_ = this->create_wall_timer(
        5s,
        [this]()
        {
          if (!motion_complete_)
          {
            RCLCPP_WARN(this->get_logger(),
                        "Motion timeout, proceeding to next step");
            motion_complete_ = true;
            this->demo_step_++;
            this->timer_ = this->create_wall_timer(
                1s, std::bind(&MotionDemoClient::RunDemo, this));
          }
        });
  }

  void StatusCallback(const hypa_msgs::msg::MotionStatus::SharedPtr _status)
  {
    if (!command_sent_)
    {
      return;
    }

    // 打印状态信息
    std::stringstream ss;
    ss << "Status: Executing=" << (_status->is_executing ? "true" : "false")
       << ", Progress=" << std::fixed << std::setprecision(1)
       << _status->progress << "% | Positions: [";
    for (size_t i = 0; i < _status->current_positions.size(); ++i)
    {
      if (i > 0) ss << ", ";
      ss << _status->current_positions[i];
    }
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

    // 检查运动是否完成
    if (_status->is_executing == false && motion_complete_ == false)
    {
      motion_complete_ = true;
      RCLCPP_INFO(this->get_logger(), "Motion completed");

      // 延迟1秒后执行下一步
      this->timer_ = this->create_wall_timer(1s,
                                             [this]()
                                             {
                                               this->demo_step_++;
                                               this->RunDemo();
                                             });
    }
  }
};

}  // namespace hypa_demos

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<hypa_demos::MotionDemoClient>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
