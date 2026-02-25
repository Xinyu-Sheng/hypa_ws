#ifndef ZMOTION_WRAPPER_HPP
#define ZMOTION_WRAPPER_HPP

#include <string>
#include <vector>
#include <optional>
#include "hypa_hardware/zmotion.h"
#include "hypa_hardware/zmcaux.h"

namespace hypa_hardware
{

/**
 * @brief ZMotion SDK 的 C++ 封装类
 *
 * 提供面向对象的接口，封装 ZMC432 控制器的底层 API 调用。
 * 所有方法线程安全（假设单线程调用，由 MotionController 保证）。
 */
class ZMotionWrapper
{
 public:
  ZMotionWrapper();
  ~ZMotionWrapper();

  // 禁止拷贝
  ZMotionWrapper(const ZMotionWrapper&) = delete;
  ZMotionWrapper& operator=(const ZMotionWrapper&) = delete;

  /**
   * @brief 连接到 ZMC432 控制器
   * @param ip 控制器 IP 地址
   * @return 成功返回 true，失败返回错误信息
   */
  std::optional<std::string> connect(const std::string& ip);

  /**
   * @brief 断开连接
   */
  void disconnect();

  /**
   * @brief 检查是否已连接
   * @return 连接状态
   */
  bool is_connected() const { return handle_ != nullptr; }

  /**
   * @brief 设置轴的脉冲当量（units）
   * @param axis 轴号（0-31）
   * @param units 脉冲当量（物理单位/脉冲），例如 0.001 表示 1 脉冲 = 0.001mm
   * @return 成功返回空，失败返回错误信息
   */
  std::optional<std::string> set_units(int axis, double units);

  /**
   * @brief 设置轴速度
   * @param axis 轴号
   * @param speed 速度（物理单位/秒）
   * @return
   */
  std::optional<std::string> set_speed(int axis, double speed);

  /**
   * @brief 设置轴加速度
   * @param axis 轴号
   * @param accel 加速度（物理单位/秒²）
   * @return
   */
  std::optional<std::string> set_acceleration(int axis, double accel);

  /**
   * @brief 设置轴减速度
   * @param axis 轴号
   * @param decel 减速度（物理单位/秒²）
   * @return
   */
  std::optional<std::string> set_deceleration(int axis, double decel);

  /**
   * @brief 单轴绝对位置运动
   * @param axis 轴号
   * @param position 目标位置（物理单位）
   * @return
   */
  std::optional<std::string> move_absolute(int axis, double position);

  /**
   * @brief 单轴相对位置运动
   * @param axis 轴号
   * @param distance 相对距离（物理单位）
   * @return
   */
  std::optional<std::string> move_relative(int axis, double distance);

  /**
   * @brief 单轴连续速度运动（JOG）
   * @param axis 轴号
   * @param velocity 速度（物理单位/秒），正负方向
   * @return
   */
  std::optional<std::string> move_velocity(int axis, double velocity);

  /**
   * @brief 多轴直线插补绝对运动
   * @param axes 轴号列表
   * @param positions 各轴目标位置（物理单位）
   * @return
   */
  std::optional<std::string> move_line_absolute(
      const std::vector<int>& axes, const std::vector<double>& positions);

  /**
   * @brief 多轴直线插补相对运动
   * @param axes 轴号列表
   * @param distances 各轴相对距离（物理单位）
   * @return
   */
  std::optional<std::string> move_line_relative(
      const std::vector<int>& axes, const std::vector<double>& distances);

  /**
   * @brief 多轴圆弧插补绝对运动（3点定圆）
   * @param axes 轴号列表（必须 2 或 3 轴）
   * @param positions 各轴目标位置（物理单位）
   * @param circular_params 圆弧参数：
   *       2D: [center_x, center_y, end_angle]
   *       3D: [center_x, center_y, center_z, end_angle, end_pitch]
   * @return
   */
  std::optional<std::string> move_circular_absolute(
      const std::vector<int>& axes, const std::vector<double>& positions,
      const std::vector<double>& circular_params);

  /**
   * @brief 多轴螺旋插补绝对运动（3轴）
   * @param axes 轴号列表（必须 3 轴）
   * @param positions 各轴目标位置 [x_end, y_end, z_end]
   * @param spiral_params 螺旋参数 [radius, pitch, turns, end_angle]
   * @return
   */
  std::optional<std::string> move_spiral_absolute(
      const std::vector<int>& axes, const std::vector<double>& positions,
      const std::vector<double>& spiral_params);

  /**
   * @brief 椭圆插补绝对运动（2轴）
   * @param axes 轴号列表（必须 2 轴）
   * @param positions 各轴目标位置
   * @param eclipse_params 椭圆参数 [center_x, center_y, major_axis, minor_axis,
   * start_angle, end_angle]
   * @return
   */
  std::optional<std::string> move_eclipse_absolute(
      const std::vector<int>& axes, const std::vector<double>& positions,
      const std::vector<double>& eclipse_params);

  /**
   * @brief 空间圆弧+螺旋插补绝对运动（3轴）
   * @param axes 轴号列表（必须 3 轴）
   * @param positions 各轴目标位置
   * @param spherical_params 空间圆弧参数 [center_x, center_y, center_z, radius,
   * start_theta, end_theta, start_phi, end_phi]
   * @return
   */
  std::optional<std::string> move_spherical_absolute(
      const std::vector<int>& axes, const std::vector<double>& positions,
      const std::vector<double>& spherical_params);

  /**
   * @brief 缓冲一个运动点（用于连续轨迹流式执行）
   * @param axes 轴号列表
   * @param positions 各轴目标位置
   * @return
   */
  std::optional<std::string> buffer_move(const std::vector<int>& axes,
                                         const std::vector<double>& positions);

  /**
   * @brief 启动连续轨迹模式
   * @return
   */
  std::optional<std::string> start_continuous();

  /**
   * @brief 停止连续轨迹模式
   * @return
   */
  std::optional<std::string> stop_continuous();

  /**
   * @brief 读取轴的规划位置（DPOS）
   * @param axis 轴号
   * @return 位置（物理单位），失败返回 optional 空值
   */
  std::optional<double> get_position(int axis);

  /**
   * @brief 读取轴的反馈位置（MPOS）
   * @param axis 轴号
   * @return 位置（物理单位），失败返回 optional 空值
   */
  std::optional<double> get_feedback(int axis);

  /**
   * @brief 读取轴的当前速度
   * @param axis 轴号
   * @return 速度（物理单位/秒），失败返回 optional 空值
   */
  std::optional<double> get_speed(int axis);

  /**
   * @brief 读取轴状态字
   * @param axis 轴号
   * @return 状态字（AXISSTATUS 位域），失败返回 optional 空值
   */
  std::optional<uint32_t> get_axis_status(int axis);

  /**
   * @brief 检查轴是否在运动
   * @param axis 轴号
   * @return true 如果轴正在运动
   */
  bool is_axis_moving(int axis);

  /**
   * @brief 停止所有轴运动
   * @return
   */
  std::optional<std::string> stop_all();

  /**
   * @brief 紧急停止（立即停止）
   * @return
   */
  std::optional<std::string> emergency_stop();

 private:
  ZMC_HANDLE handle_{nullptr};
  std::string last_error_;

  /**
   * @brief 将物理单位位置转换为脉冲单位
   * @param axis 轴号
   * @param physical_position 物理位置
   * @return 脉冲位置（整数）
   */
  int64_t physical_to_pulses(int axis, double physical_position);

  /**
   * @brief 将脉冲单位位置转换为物理单位
   * @param axis 轴号
   * @param pulses 脉冲位置
   * @return 物理位置
   */
  double pulses_to_physical(int axis, int64_t pulses);

  /**
   * @brief 检查并更新轴配置缓存
   * @param axis 轴号
   * @return
   */
  std::optional<std::string> ensure_axis_configured(int axis);

  /**
   * @brief 获取轴当前 units 参数
   * @param axis 轴号
   * @return units 值
   */
  double get_current_units(int axis);

 private:
  // 轴配置缓存
  struct AxisConfigCache
  {
    double units = 1.0;  // 默认 1.0（脉冲单位）
    bool configured = false;
  };
  std::vector<AxisConfigCache> axis_configs_;
};

}  // namespace hypa_hardware

#endif  // ZMOTION_WRAPPER_HPP
