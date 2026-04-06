#include <QDebug>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;

builtin_interfaces::msg::Time makeStamp(double _time_sec)
{
  builtin_interfaces::msg::Time stamp;
  const int64_t total_nanoseconds =
      static_cast<int64_t>(_time_sec * 1000000000.0);
  stamp.sec = static_cast<int32_t>(total_nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(total_nanoseconds % 1000000000LL);
  return stamp;
}

void setQuaternionFromRPY(double _roll, double _pitch, double _yaw, double &_x,
                          double &_y, double &_z, double &_w)
{
  const double half_roll = _roll * 0.5;
  const double half_pitch = _pitch * 0.5;
  const double half_yaw = _yaw * 0.5;

  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);

  _w = cr * cp * cy + sr * sp * sy;
  _x = sr * cp * cy - cr * sp * sy;
  _y = cr * sp * cy + sr * cp * sy;
  _z = cr * cp * sy - sr * sp * cy;
}

sensor_msgs::msg::Imu makeImuMessage(const std::string &_frame_id,
                                     double _time_sec, double _phase_offset)
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = makeStamp(_time_sec);
  message.header.frame_id = _frame_id;

  const double roll = 0.08 * std::sin(_time_sec * 0.7 + _phase_offset);
  const double pitch = 0.06 * std::cos(_time_sec * 0.5 + _phase_offset);
  const double yaw = 0.16 * std::sin(_time_sec * 0.35 + _phase_offset * 0.4);

  setQuaternionFromRPY(roll, pitch, yaw, message.orientation.x,
                       message.orientation.y, message.orientation.z,
                       message.orientation.w);

  message.angular_velocity.x = 0.45 * std::cos(_time_sec * 0.9 + _phase_offset);
  message.angular_velocity.y = 0.35 * std::sin(_time_sec * 1.2 + _phase_offset);
  message.angular_velocity.z = 0.25 * std::cos(_time_sec * 0.6 + _phase_offset);

  message.linear_acceleration.x =
      1.5 * std::sin(_time_sec * 1.0 + _phase_offset);
  message.linear_acceleration.y =
      1.2 * std::cos(_time_sec * 0.8 + _phase_offset);
  message.linear_acceleration.z =
      9.81 + 0.5 * std::sin(_time_sec * 0.5 + _phase_offset);

  for (size_t i = 0; i < 9; ++i)
  {
    message.orientation_covariance[i] = 0.001 * static_cast<double>(i + 1);
    message.angular_velocity_covariance[i] = 0.002 * static_cast<double>(i + 1);
    message.linear_acceleration_covariance[i] =
        0.003 * static_cast<double>(i + 1);
  }

  return message;
}

sensor_msgs::msg::JointState makeJointStateMessage(double _time_sec)
{
  sensor_msgs::msg::JointState message;
  message.header.stamp = makeStamp(_time_sec);
  message.name = {"joint_1", "joint_2", "joint_3", "joint_4"};
  message.position.resize(message.name.size());
  message.velocity.resize(message.name.size());
  message.effort.resize(message.name.size());

  for (size_t i = 0; i < message.name.size(); ++i)
  {
    const double phase = _time_sec * (0.5 + 0.15 * static_cast<double>(i));
    const double gain = static_cast<double>(i + 1);
    message.position[i] = 0.4 * gain * std::sin(phase);
    message.velocity[i] = 0.2 * gain * std::cos(phase * 1.5);
    message.effort[i] = 0.7 * gain * std::sin(phase * 0.75 + 0.2);
  }

  return message;
}
}  // namespace

class HypaDtGuiPluginTestNode : public rclcpp::Node
{
  public:
  HypaDtGuiPluginTestNode() : rclcpp::Node("hypa_dt_gui_plugin_test_node")
  {
    this->imu_front_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/hypa/imu/data", rclcpp::SensorDataQoS());
    this->imu_rear_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/hypa/imu/data1", rclcpp::SensorDataQoS());
    this->joint_state_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>(
            "/hypa/joint_states", rclcpp::SensorDataQoS());

    this->start_time_ = this->now().seconds();
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&HypaDtGuiPluginTestNode::onTimer, this));

    qInfo() << "[hypa-dt-gui-plugin-test-node] test publishers started";
  }

  private:
  void onTimer()
  {
    const double elapsed_sec = this->now().seconds() - this->start_time_;

    auto imu_front = makeImuMessage("imu_front", elapsed_sec, 0.0);
    auto imu_rear = makeImuMessage("imu_rear", elapsed_sec, kPi * 0.5);
    auto joint_state = makeJointStateMessage(elapsed_sec);

    this->imu_front_pub_->publish(imu_front);
    this->imu_rear_pub_->publish(imu_rear);
    this->joint_state_pub_->publish(joint_state);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_front_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_rear_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double start_time_ = 0.0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HypaDtGuiPluginTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
