#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hypa_msgs/action/multi_axis_motion.hpp"

using namespace std::chrono_literals;

namespace hypa_demos
{

class MotionDemoClient : public rclcpp::Node
{
 public:
  using MultiAxisMotion = hypa_msgs::action::MultiAxisMotion;
  using GoalHandleMultiAxisMotion =
      rclcpp_action::ClientGoalHandle<MultiAxisMotion>;

  explicit MotionDemoClient(const rclcpp::NodeOptions& _options)
      : Node("motion_demo_client", _options)
  {
    this->client_ptr_ = rclcpp_action::create_client<MultiAxisMotion>(
        this, "motion/multi_axis_move");

    this->timer_ = this->create_wall_timer(
        500ms, std::bind(&MotionDemoClient::RunDemo, this));

    RCLCPP_INFO(this->get_logger(), "Motion Demo Client initialized.");
  }

 private:
  rclcpp_action::Client<MultiAxisMotion>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  int demo_step_ = 0;
  bool goal_in_progress_ = false;

  void RunDemo()
  {
    if (this->goal_in_progress_)
    {
      return;
    }

    this->timer_->cancel();

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

    if (!this->WaitForServer()) return;

    auto goal_msg = MultiAxisMotion::Goal();
    goal_msg.axes = {0};
    goal_msg.positions = {100.0};
    goal_msg.velocities = {20.0};
    goal_msg.motion_type = 1;  // SINGLE_AXIS

    this->SendGoal(goal_msg);
    this->demo_step_ = 1;
  }

  void TestInterpolatedMotion()
  {
    RCLCPP_INFO(
        this->get_logger(),
        "--- Step 2: Dual Axis Interpolation (Axis 0,1 to 50.0, 50.0) ---");

    if (!this->WaitForServer()) return;

    auto goal_msg = MultiAxisMotion::Goal();
    goal_msg.axes = {0, 1};
    goal_msg.positions = {50.0, 50.0};
    goal_msg.velocities = {10.0, 10.0};
    goal_msg.motion_type = 2;         // INTERPOLATED
    goal_msg.interpolation_mode = 0;  // LINEAR

    this->SendGoal(goal_msg);
    this->demo_step_ = 2;
  }

  void TestCancelMotion()
  {
    RCLCPP_INFO(
        this->get_logger(),
        "--- Step 3: Cancellation Demo (Long move, cancel after 2s) ---");

    if (!this->WaitForServer()) return;

    auto goal_msg = MultiAxisMotion::Goal();
    goal_msg.axes = {0};
    goal_msg.positions = {500.0};
    goal_msg.velocities = {5.0};  // Slow move
    goal_msg.motion_type = 1;

    auto send_goal_options =
        rclcpp_action::Client<MultiAxisMotion>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const GoalHandleMultiAxisMotion::SharedPtr& _goal_handle)
    {
      if (!_goal_handle)
      {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(),
                    "Goal accepted by server, waiting 2s before canceling...");

        // Wait 2 seconds then cancel
        rclcpp::sleep_for(2s);
        RCLCPP_INFO(this->get_logger(), "Sending cancel request...");
        this->client_ptr_->async_cancel_goal(_goal_handle);
      }
    };

    send_goal_options.feedback_callback =
        std::bind(&MotionDemoClient::FeedbackCallback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &MotionDemoClient::ResultCallback, this, std::placeholders::_1);

    this->goal_in_progress_ = true;
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    this->demo_step_ = 3;
  }

  bool WaitForServer()
  {
    if (!this->client_ptr_->wait_for_action_server(5s))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
      return false;
    }
    return true;
  }

  void SendGoal(const MultiAxisMotion::Goal& _goal_msg)
  {
    auto send_goal_options =
        rclcpp_action::Client<MultiAxisMotion>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const GoalHandleMultiAxisMotion::SharedPtr& _goal_handle)
    {
      if (!_goal_handle)
      {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(),
                    "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback =
        std::bind(&MotionDemoClient::FeedbackCallback, this,
                  std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback = std::bind(
        &MotionDemoClient::ResultCallback, this, std::placeholders::_1);

    this->goal_in_progress_ = true;
    this->client_ptr_->async_send_goal(_goal_msg, send_goal_options);
  }

  void FeedbackCallback(
      GoalHandleMultiAxisMotion::SharedPtr,
      const std::shared_ptr<const MultiAxisMotion::Feedback> _feedback)
  {
    std::stringstream ss;
    ss << "Progress: " << std::fixed << std::setprecision(1)
       << _feedback->progress << "% | Pos: [";
    for (size_t i = 0; i < _feedback->current_positions.size(); ++i)
    {
      ss << (i > 0 ? ", " : "") << _feedback->current_positions[i];
    }
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }

  void ResultCallback(const GoalHandleMultiAxisMotion::WrappedResult& _result)
  {
    this->goal_in_progress_ = false;

    switch (_result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted: %s",
                     _result.result->error_message.c_str());
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }

    // Schedule next step
    this->timer_ = this->create_wall_timer(
        1s, std::bind(&MotionDemoClient::RunDemo, this));
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
