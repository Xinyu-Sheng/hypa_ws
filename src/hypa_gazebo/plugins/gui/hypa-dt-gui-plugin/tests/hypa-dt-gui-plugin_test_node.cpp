#include <QDebug>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <random>

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

    // initialize RNG and per-sensor/joint random parameters
    std::random_device rd;
    this->rng_ = std::mt19937(rd());

    std::uniform_real_distribution<double> amp_dist(0.04, 0.12);
    std::uniform_real_distribution<double> amp_yaw_dist(0.08, 0.2);
    std::uniform_real_distribution<double> freq_dist(0.2, 1.0);
    std::uniform_real_distribution<double> phase_dist(0.0, kPi * 2.0);
    std::uniform_real_distribution<double> noise_small(0.0005, 0.01);

    // front IMU params
    imu_front_params_.amp_roll = amp_dist(rng_);
    imu_front_params_.amp_pitch = amp_dist(rng_);
    imu_front_params_.amp_yaw = amp_yaw_dist(rng_);
    imu_front_params_.freq_roll = freq_dist(rng_);
    imu_front_params_.freq_pitch = freq_dist(rng_);
    imu_front_params_.freq_yaw = freq_dist(rng_);
    imu_front_params_.phase_roll = phase_dist(rng_);
    imu_front_params_.phase_pitch = phase_dist(rng_);
    imu_front_params_.phase_yaw = phase_dist(rng_);
    imu_front_params_.noise_ori_std = noise_small(rng_);
    imu_front_params_.noise_angvel_std = noise_small(rng_);
    imu_front_params_.noise_linacc_std = noise_small(rng_);
    imu_front_params_.ou_theta = 1.0 + 0.5 * freq_dist(rng_);
    imu_front_params_.ou_sigma = 0.02 + noise_small(rng_);
    imu_front_params_.ou_mu_ori = 0.0;
    imu_front_params_.ou_mu_angvel = 0.0;
    imu_front_params_.ou_mu_linacc_z = 9.81;

    // rear IMU params (independent)
    imu_rear_params_.amp_roll = amp_dist(rng_);
    imu_rear_params_.amp_pitch = amp_dist(rng_);
    imu_rear_params_.amp_yaw = amp_yaw_dist(rng_);
    imu_rear_params_.freq_roll = freq_dist(rng_);
    imu_rear_params_.freq_pitch = freq_dist(rng_);
    imu_rear_params_.freq_yaw = freq_dist(rng_);
    imu_rear_params_.phase_roll = phase_dist(rng_);
    imu_rear_params_.phase_pitch = phase_dist(rng_);
    imu_rear_params_.phase_yaw = phase_dist(rng_);
    imu_rear_params_.noise_ori_std = noise_small(rng_);
    imu_rear_params_.noise_angvel_std = noise_small(rng_);
    imu_rear_params_.noise_linacc_std = noise_small(rng_);
    imu_rear_params_.ou_theta = 0.8 + 0.6 * freq_dist(rng_);
    imu_rear_params_.ou_sigma = 0.01 + noise_small(rng_);
    imu_rear_params_.ou_mu_ori = 0.0;
    imu_rear_params_.ou_mu_angvel = 0.0;
    imu_rear_params_.ou_mu_linacc_z = 9.81;

    // joints
    joint_params_.resize(4);
    std::uniform_real_distribution<double> joint_amp(0.2, 0.6);
    std::uniform_real_distribution<double> joint_freq(0.3, 1.2);
    for (size_t i = 0; i < joint_params_.size(); ++i)
    {
      joint_params_[i].amp_position = joint_amp(rng_);
      joint_params_[i].freq_position = joint_freq(rng_);
      joint_params_[i].phase_position = phase_dist(rng_);
      joint_params_[i].noise_pos_std = noise_small(rng_);
      joint_params_[i].noise_vel_std = noise_small(rng_);
      joint_params_[i].noise_effort_std = noise_small(rng_);
      joint_params_[i].ou_theta = 0.7 + 0.6 * joint_freq(rng_);
      joint_params_[i].ou_sigma = 0.01 + noise_small(rng_);
      joint_params_[i].ou_mu_pos = 0.0;
    }

    // initialize OU states
    joint_states_.resize(joint_params_.size());
    std::normal_distribution<double> initn(0.0, 1.0);
    for (size_t i = 0; i < joint_states_.size(); ++i)
    {
      joint_states_[i].pos = joint_params_[i].ou_mu_pos + 0.1 * initn(rng_);
      joint_states_[i].vel = 0.01 * initn(rng_);
      joint_states_[i].eff = 0.01 * initn(rng_);
    }

    for (int i = 0; i < 3; ++i)
    {
      imu_front_ori_state_[i] = 0.01 * initn(rng_);
      imu_front_angvel_state_[i] = 0.01 * initn(rng_);
      imu_front_linacc_state_[i] = (i == 2) ? 9.81 + 0.1 * initn(rng_) : 0.01 * initn(rng_);
      imu_rear_ori_state_[i] = 0.01 * initn(rng_);
      imu_rear_angvel_state_[i] = 0.01 * initn(rng_);
      imu_rear_linacc_state_[i] = (i == 2) ? 9.81 + 0.1 * initn(rng_) : 0.01 * initn(rng_);
    }

    this->last_time_ = this->now().seconds();

    this->start_time_ = this->now().seconds();
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&HypaDtGuiPluginTestNode::onTimer, this));

    qInfo() << "[hypa-dt-gui-plugin-test-node] test publishers started";
  }

  private:
  struct ImuParams
  {
    double amp_roll;
    double amp_pitch;
    double amp_yaw;
    double freq_roll;
    double freq_pitch;
    double freq_yaw;
    double phase_roll;
    double phase_pitch;
    double phase_yaw;
    double noise_ori_std;
    double noise_angvel_std;
    double noise_linacc_std;
    // OU process params
    double ou_theta;
    double ou_sigma;
    double ou_mu_ori;
    double ou_mu_angvel;
    double ou_mu_linacc_z;
  };

  struct JointParams
  {
    double amp_position;
    double freq_position;
    double phase_position;
    double noise_pos_std;
    double noise_vel_std;
    double noise_effort_std;
    double ou_theta;
    double ou_sigma;
    double ou_mu_pos;
  };

  // Node-local RNG and per-sensor/joint parameters to make outputs independent
  std::mt19937 rng_;
  ImuParams imu_front_params_;
  ImuParams imu_rear_params_;
  std::vector<JointParams> joint_params_;
  // OU states for IMUs: orientation (r,p,y), angvel (x,y,z), linacc (x,y,z)
  std::array<double, 3> imu_front_ori_state_{};
  std::array<double, 3> imu_front_angvel_state_{};
  std::array<double, 3> imu_front_linacc_state_{};
  std::array<double, 3> imu_rear_ori_state_{};
  std::array<double, 3> imu_rear_angvel_state_{};
  std::array<double, 3> imu_rear_linacc_state_{};
  // Joint states
  struct JointState
  {
    double pos;
    double vel;
    double eff;
  };
  std::vector<JointState> joint_states_;
  double last_time_ = 0.0;

  // Generate IMU message using node-local independent random params and OU states
  sensor_msgs::msg::Imu makeImuMessageNode(const std::string &_frame_id,
                                          const ImuParams &_p,
                                          const std::array<double, 3> &_ori_state,
                                          const std::array<double, 3> &_angvel_state,
                                          const std::array<double, 3> &_linacc_state)
  {
    sensor_msgs::msg::Imu message;
    // header stamp will be set by caller
    message.header.frame_id = _frame_id;

    // deterministic low-frequency components + gaussian noise
    std::normal_distribution<double> noise_ori(0.0, _p.noise_ori_std);
    std::normal_distribution<double> noise_av(0.0, _p.noise_angvel_std);
    std::normal_distribution<double> noise_la(0.0, _p.noise_linacc_std);

    const double roll = _ori_state[0] + noise_ori(rng_);
    const double pitch = _ori_state[1] + noise_ori(rng_);
    const double yaw = _ori_state[2] + noise_ori(rng_);

    setQuaternionFromRPY(roll, pitch, yaw, message.orientation.x,
                         message.orientation.y, message.orientation.z,
                         message.orientation.w);

    message.angular_velocity.x = _angvel_state[0] + noise_av(rng_);
    message.angular_velocity.y = _angvel_state[1] + noise_av(rng_);
    message.angular_velocity.z = _angvel_state[2] + noise_av(rng_);

    message.linear_acceleration.x = _linacc_state[0] + noise_la(rng_);
    message.linear_acceleration.y = _linacc_state[1] + noise_la(rng_);
    message.linear_acceleration.z = _linacc_state[2] + noise_la(rng_);

    // covariance with small random variations but independent entries
    std::uniform_real_distribution<double> cov_jitter(0.0005, 0.005);
    for (size_t i = 0; i < 9; ++i)
    {
      message.orientation_covariance[i] = (i % 4 == 0) ? cov_jitter(rng_) : 0.0;
      message.angular_velocity_covariance[i] = (i % 4 == 0) ? cov_jitter(rng_) * 2.0 : 0.0;
      message.linear_acceleration_covariance[i] = (i % 4 == 0) ? cov_jitter(rng_) * 3.0 : 0.0;
    }

    return message;
  }

  sensor_msgs::msg::JointState makeJointStateMessageNode(double _time_sec)
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = makeStamp(_time_sec);
    message.name = {"joint_1", "joint_2", "joint_3", "joint_4"};
    message.position.resize(message.name.size());
    message.velocity.resize(message.name.size());
    message.effort.resize(message.name.size());

    for (size_t i = 0; i < message.name.size(); ++i)
    {
      const auto &p = joint_params_[i];
      const auto &s = joint_states_[i];
      std::normal_distribution<double> noise_pos(0.0, p.noise_pos_std);
      std::normal_distribution<double> noise_vel(0.0, p.noise_vel_std);
      std::normal_distribution<double> noise_eff(0.0, p.noise_effort_std);

      message.position[i] = s.pos + noise_pos(rng_);
      message.velocity[i] = s.vel + noise_vel(rng_);
      message.effort[i] = s.eff + noise_eff(rng_);
    }

    return message;
  }

  // Update OU states for IMUs and joints using dt
  void updateStates(double dt)
  {
    if (dt <= 0.0) return;
    std::normal_distribution<double> norm(0.0, 1.0);

    auto ou_step = [&](double &x, double theta, double mu, double sigma)
    {
      x += theta * (mu - x) * dt + sigma * std::sqrt(dt) * norm(rng_);
    };

    // IMU front
    for (int i = 0; i < 3; ++i)
    {
      ou_step(imu_front_ori_state_[i], imu_front_params_.ou_theta, imu_front_params_.ou_mu_ori, imu_front_params_.ou_sigma);
      ou_step(imu_front_angvel_state_[i], imu_front_params_.ou_theta, imu_front_params_.ou_mu_angvel, imu_front_params_.ou_sigma);
      double mu_la = (i == 2) ? imu_front_params_.ou_mu_linacc_z : 0.0;
      ou_step(imu_front_linacc_state_[i], imu_front_params_.ou_theta, mu_la, imu_front_params_.ou_sigma);
    }

    // IMU rear
    for (int i = 0; i < 3; ++i)
    {
      ou_step(imu_rear_ori_state_[i], imu_rear_params_.ou_theta, imu_rear_params_.ou_mu_ori, imu_rear_params_.ou_sigma);
      ou_step(imu_rear_angvel_state_[i], imu_rear_params_.ou_theta, imu_rear_params_.ou_mu_angvel, imu_rear_params_.ou_sigma);
      double mu_la = (i == 2) ? imu_rear_params_.ou_mu_linacc_z : 0.0;
      ou_step(imu_rear_linacc_state_[i], imu_rear_params_.ou_theta, mu_la, imu_rear_params_.ou_sigma);
    }

    // joints
    for (size_t i = 0; i < joint_states_.size(); ++i)
    {
      auto &s = joint_states_[i];
      const auto &p = joint_params_[i];
      ou_step(s.pos, p.ou_theta, p.ou_mu_pos, p.ou_sigma);
      ou_step(s.vel, p.ou_theta, 0.0, p.ou_sigma * 0.5);
      ou_step(s.eff, p.ou_theta, 0.0, p.ou_sigma * 0.8);
    }
  }

  void onTimer()
  {
    const double now_sec = this->now().seconds();
    const double dt = now_sec - this->last_time_;
    this->last_time_ = now_sec;

    // update OU states
    updateStates(dt);

    auto imu_front = makeImuMessageNode("imu_front", this->imu_front_params_, imu_front_ori_state_, imu_front_angvel_state_, imu_front_linacc_state_);
    imu_front.header.stamp = makeStamp(now_sec - this->start_time_);
    auto imu_rear = makeImuMessageNode("imu_rear", this->imu_rear_params_, imu_rear_ori_state_, imu_rear_angvel_state_, imu_rear_linacc_state_);
    imu_rear.header.stamp = makeStamp(now_sec - this->start_time_);
    auto joint_state = makeJointStateMessageNode(now_sec - this->start_time_);

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
