# 多轴运动状态消息
# 发布当前运动状态和反馈信息

float64[] current_positions        # 各轴的规划位置（DPOS，单位：物理单位）
float64[] feedback_positions       # 各轴的反馈位置（MPOS，单位：物理单位）
float64[] current_velocities       # 各轴的当前速度（物理单位/秒）
uint32[] axis_statuses             # 各轴的状态字（ZMotion AXISSTATUS 位域）
int32[] axis_atypes                # 各轴的类型（ATYPE）
bool[] axis_enabled               # 各轴的使能状态
float64 progress                   # 整体运动进度（0-100%），基于各轴剩余距离加权计算
int32[] executing_axis             # 当前执行命令涉及的轴列表（无执行时为空）
bool is_executing                  # 控制器是否正在执行运动
