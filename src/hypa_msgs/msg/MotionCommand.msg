# 多轴运动命令消息
# 支持单轴独立运动和多轴插补运动

int32[] axes                       # 轴号列表（0-31）
float64[] positions                # 目标位置（物理单位：mm 或 degree）
float64[] velocities               # 运动速度（物理单位/秒）
float64[] accelerations            # 加速度（物理单位/秒²），可选，缺省使用轴配置
float64[] decelerations            # 减速度（物理单位/秒²），可选，缺省使用轴配置
uint8 motion_type                  # 运动类型：
                                  #   1 = SINGLE_AXIS（各轴独立运动，不同步）
                                  #   2 = INTERPOLATED（多轴插补，同步起停）
                                  #   3 = CONTINUOUS_TRAJECTORY（连续轨迹，流式缓冲）
uint8 interpolation_mode           # 插补模式（仅 motion_type=2 或 3 时有效）：
                                  #   0 = LINEAR（直线插补）
                                  #   1 = CIRCULAR（圆弧插补，3点定圆）
                                  #   2 = SPIRAL（螺旋插补，3轴）
                                  #   3 = ECLIPSE（椭圆插补）
                                  #   4 = SPHERICAL（空间圆弧+螺旋插补）
float64[] circular_params          # 圆弧插补的额外参数（圆心坐标或中间点），格式取决于插补模式
                                  # LINEAR: 忽略
                                  # CIRCULAR (2D): [center_x, center_y, end_angle]
                                  # CIRCULAR (3D): [center_x, center_y, center_z, end_angle, end_pitch]
                                  # SPIRAL: [radius, pitch, turns, end_angle]
                                  # ECLIPSE: [center_x, center_y, major_axis, minor_axis, start_angle, end_angle]
                                  # SPHERICAL: [center_x, center_y, center_z, radius, start_theta, end_theta, start_phi, end_phi]
float64 wait_time                  # 运动完成后等待时间（毫秒），可选，缺省 0
