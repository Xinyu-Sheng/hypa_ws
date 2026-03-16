#include <memory>

#include "hypa_msgs/msg/motion_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class ServoJointForwardNode : public rclcpp::Node
{
  public:
  ServoJointForwardNode() : Node("servo_joint_forward")
  {
    pub_ = this->create_publisher<std_msgs::msg::Float64>("/servo_joint", 10);

    sub_ = this->create_subscription<hypa_msgs::msg::MotionStatus>(
        "/hypa/motion_status", 10,
        std::bind(&ServoJointForwardNode::onStatus, this,
                  std::placeholders::_1));
  }

  private:
  void onStatus(const hypa_msgs::msg::MotionStatus::SharedPtr msg)
  {
    if (msg->feedback_positions.empty())
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "motion_status.feedback_positions is empty; cannot forward");
      return;
    }

    std_msgs::msg::Float64 out;
    out.data = msg->feedback_positions[0];
    pub_->publish(out);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<hypa_msgs::msg::MotionStatus>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoJointForwardNode>());
  rclcpp::shutdown();
  return 0;
}
