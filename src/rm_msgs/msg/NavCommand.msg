# 决策 -> 导航：任务级控制指令（接口约定见 2.md §3）
std_msgs/Header header

# 导航模式
uint8 MODE_STOP = 0   # 急停：立即零速
uint8 MODE_GOTO = 1   # 前往目标点
uint8 MODE_HOLD = 2   # 原地驻留：在目标点做位置保持
uint8 mode

# 任务编号，仅用于日志/状态显示
uint8 TASK_IDLE = 0
uint8 TASK_SUPPLY = 1
uint8 TASK_CENTRAL_HIGH = 2
uint8 TASK_TRAPEZOID_HIGH = 3
uint8 TASK_BUNKER = 4
uint8 task_id

float32 x           # 目标点 x, m（global_frame 系）
float32 y           # 目标点 y, m（global_frame 系）
float32 yaw         # 目标朝向 rad；NaN 表示不约束朝向
float32 spin_rate   # 小陀螺角速度 deg/s（逆时针为正）；0 表示不转
float32 stay_sec    # mode=HOLD 的期望保持时长 s（供导航侧超时保护）
float32 radius      # 到位/保持半径 m（该点位的判定半径）；NaN = 用导航默认 arrive_radius
