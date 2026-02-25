#ifndef MOTION_CONTROLLER_HPP
#define MOTION_CONTROLLER_HPP

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <vector>
#include <map>
#include <atomic>
#include <optional>
#include "hypa_hardware/zmotion_wrapper.hpp"

namespace hypa_hardware
{

/**
 * @brief 运动控制器
 *
 * 管理多轴运动协调、命令缓冲队列和执行线程。
 * 支持单轴独立运动、多轴插补运动、连续轨迹流式缓冲。
 */
class MotionController
{
 public:
  enum MotionType : uint8_t
  {
    SINGLE_AXIS = 1,
    INTERPOLATED = 2,
    CONTINUOUS_TRAJECTORY = 3
  };

  enum InterpolationMode : uint8_t
  {
    LINEAR = 0,
    CIRCULAR = 1,
    SPIRAL = 2,
    ECLIPSE = 3,
    SPHERICAL = 4
  };

  struct MotionCommand
  {
    MotionType motion_type = SINGLE_AXIS;
    InterpolationMode interpolation_mode = LINEAR;
    std::vector<int> axes;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
    std::vector<double> decelerations;
    std::vector<double> circular_params;
    double wait_time = 0.0;
  };

  struct AxisStatus
  {
    double position = 0.0;     // 规划位置
    double feedback = 0.0;     // 反馈位置
    double speed = 0.0;        // 当前速度
    uint32_t status_word = 0;  // 状态字
    bool moving = false;       // 是否在运动
    bool error = false;        // 是否有错误
  };

  struct ControllerStatus
  {
    std::map<int, AxisStatus> axis_statuses;
    bool executing = false;
    bool stop_requested = false;
    double progress = 0.0;  // 0-100%
  };

  MotionController();
  ~MotionController();

  // 禁止拷贝
  MotionController(const MotionController&) = delete;
  MotionController& operator=(const MotionController&) = delete;

  /**
   * @brief 初始化控制器，连接 ZMC432
   * @param controller_ip 控制器 IP 地址
   * @return 成功返回 true，失败返回错误信息
   */
  std::optional<std::string> initialize(const std::string& _controller_ip);

  /**
   * @brief 配置轴参数
   * @param axis 轴号
   * @param units 脉冲当量
   * @param speed 最大速度
   * @param accel 加速度
   * @param decel 减速度
   * @return
   */
  std::optional<std::string> configure_axis(int _axis, double _units,
                                            double _speed, double _accel,
                                            double _decel);

  /**
   * @brief 启动执行线程
   * @return
   */
  bool start();

  /**
   * @brief 停止执行线程并清空缓冲
   * @return
   */
  void stop();

  /**
   * @brief 提交运动命令到缓冲队列
   * @param cmd 运动命令
   * @return 如果成功入队返回 true，否则 false（队列已满或控制器未就绪）
   */
  bool queue_motion(const MotionCommand& cmd);

  /**
   * @brief 获取当前控制器状态
   * @return 状态结构体
   */
  ControllerStatus get_current_status() const;

  /**
   * @brief 请求取消当前运动
   */
  void cancel_current_motion();

  /**
   * @brief 检查是否有运动正在执行
   * @return
   */
  bool is_executing() const { return executing_; }

  /**
   * @brief 等待当前运动完成（阻塞）
   * @param timeout_ms 超时时间（毫秒），0 表示无限等待
   * @return 成功完成返回 true，超时或取消返回 false
   */
  bool wait_for_completion(int _timeout_ms = 0);

 private:
  std::unique_ptr<ZMotionWrapper> zmotion_;
  std::thread execution_thread_;
  std::mutex buffer_mutex_;
  std::condition_variable buffer_cv_;
  std::deque<MotionCommand> command_buffer_;
  std::atomic<bool> running_{false};
  std::atomic<bool> executing_{false};
  std::atomic<bool> cancel_requested_{false};

  // 轴配置
  struct AxisConfig
  {
    double units = 1.0;
    double speed = 10.0;
    double acceleration = 100.0;
    double deceleration = 100.0;
    bool configured = false;
  };
  std::map<int, AxisConfig> axis_configs_;

  /**
   * @brief 执行线程主循环
   */
  void execution_loop();

  /**
   * @brief 执行单轴运动命令
   * @param cmd 运动命令
   * @return
   */
  std::optional<std::string> execute_single_axis(const MotionCommand& cmd);

  /**
   * @brief 执行插补运动命令
   * @param cmd 运动命令
   * @return
   */
  std::optional<std::string> execute_interpolated(const MotionCommand& cmd);

  /**
   * @brief 执行连续轨迹运动命令
   * @param cmd 运动命令
   * @return
   */
  std::optional<std::string> execute_continuous_trajectory(
      const MotionCommand& cmd);

  /**
   * @brief 更新各轴状态
   */
  void update_status();

  /**
   * @brief 计算整体运动进度
   * @param cmd 当前执行的命令
   * @return 进度百分比（0-100）
   */
  double calculate_progress(const MotionCommand& cmd) const;

  /**
   * @brief 检查命令是否完成
   * @param cmd 当前命令
   * @return 如果所有轴都到达目标位置则返回 true
   */
  bool is_motion_complete(const MotionCommand& cmd) const;
};

}  // namespace hypa_hardware

#endif  // MOTION_CONTROLLER_HPP
