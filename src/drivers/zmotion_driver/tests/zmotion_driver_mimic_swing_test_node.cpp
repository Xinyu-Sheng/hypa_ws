#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace
{
#if !defined(ZMOTION_TEST_MIMIC_GROUP)
  #define ZMOTION_TEST_MIMIC_GROUP 1
#endif

static_assert((ZMOTION_TEST_MIMIC_GROUP == 1) ||
                  (ZMOTION_TEST_MIMIC_GROUP == 2),
              "ZMOTION_TEST_MIMIC_GROUP must be 1 or 2");

constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultSwingRange = 0.01;
constexpr double kDefaultSwingPeriodSec = 6.0;
constexpr std::chrono::milliseconds kTimerPeriod{10};

const char *getMimicTopic()
{
#if ZMOTION_TEST_MIMIC_GROUP == 1
  return "cmd/mimic_group1";
#else
  return "cmd/mimic_group2";
#endif
}

const char *getGroupName()
{
#if ZMOTION_TEST_MIMIC_GROUP == 1
  return "mimic_group_1";
#else
  return "mimic_group_2";
#endif
}

const char *getDefaultJointName()
{
#if ZMOTION_TEST_MIMIC_GROUP == 1
  return "joint_4";
#else
  return "joint_8";
#endif
}

}  // namespace

class ZMotionMimicSwingTestNode : public rclcpp::Node
{
  public:
  explicit ZMotionMimicSwingTestNode(
      const rclcpp::NodeOptions &_options = rclcpp::NodeOptions())
      : rclcpp::Node("zmotion_mimic_swing_test_node", _options)
  {
    this->declare_parameter<double>("swing_range", kDefaultSwingRange);
    this->declare_parameter<double>("swing_period_sec", kDefaultSwingPeriodSec);
    this->declare_parameter<std::string>("joint_states_topic", "joint_states");

    const auto swing_range = this->get_parameter("swing_range").as_double();
    const auto swing_period_sec =
        this->get_parameter("swing_period_sec").as_double();
    this->joint_states_topic_ =
        this->get_parameter("joint_states_topic").as_string();

    this->swing_range_ = std::abs(swing_range);
    this->swing_period_sec_ =
        (swing_period_sec > 0.0) ? swing_period_sec : kDefaultSwingPeriodSec;
    this->angular_frequency_ = (2.0 * kPi) / this->swing_period_sec_;

    this->joint_state_sub_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            this->joint_states_topic_, rclcpp::SystemDefaultsQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr _msg)
            { this->onJointState(_msg); });

    this->mimic_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        getMimicTopic(), rclcpp::SystemDefaultsQoS());

    this->timer_ =
        this->create_wall_timer(kTimerPeriod, [this]() { this->onTimer(); });

    RCLCPP_INFO(this->get_logger(),
                "started %s targeting topic %s, joint_states topic %s, "
                "swing_range=%.6f, swing_period_sec=%.6f",
                getGroupName(), getMimicTopic(),
                this->joint_states_topic_.c_str(), this->swing_range_,
                this->swing_period_sec_);
  }

  private:
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr &_msg)
  {
    if (_msg == nullptr)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->have_initial_position_)
    {
      return;
    }

    const std::string target_joint_name = getDefaultJointName();
    for (std::size_t i = 0; i < _msg->name.size(); ++i)
    {
      if (_msg->name[i] != target_joint_name)
      {
        continue;
      }

      if (i >= _msg->position.size())
      {
        RCLCPP_WARN(this->get_logger(),
                    "joint_states contains %s but position array is too small",
                    target_joint_name.c_str());
        return;
      }

      this->initial_position_ = _msg->position[i];
      this->have_initial_position_ = true;
      this->start_time_ = this->now().seconds();
      this->last_target_position_ = this->initial_position_;
      RCLCPP_INFO(this->get_logger(), "captured initial position for %s: %.6f",
                  target_joint_name.c_str(), this->initial_position_);
      return;
    }
  }

  void onTimer()
  {
    double initial_position = 0.0;
    double start_time = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (!this->have_initial_position_)
      {
        return;
      }
      initial_position = this->initial_position_;
      start_time = this->start_time_;
    }

    const double now_sec = this->now().seconds();
    const double elapsed_sec = now_sec - start_time;
    const double target_position =
        initial_position +
        this->swing_range_ * std::sin(this->angular_frequency_ * elapsed_sec);

    std_msgs::msg::Float64 command_msg;
    command_msg.data = target_position;
    this->mimic_cmd_pub_->publish(command_msg);

    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->last_target_position_ = target_position;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mimic_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  std::string joint_states_topic_;
  double swing_range_ = kDefaultSwingRange;
  double swing_period_sec_ = kDefaultSwingPeriodSec;
  double angular_frequency_ = (2.0 * kPi) / kDefaultSwingPeriodSec;
  bool have_initial_position_ = false;
  double initial_position_ = 0.0;
  double start_time_ = 0.0;
  double last_target_position_ = std::numeric_limits<double>::quiet_NaN();
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ZMotionMimicSwingTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
