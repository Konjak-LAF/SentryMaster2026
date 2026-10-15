# 决策 -> uart_driver：姿态指令
std_msgs/Header header

# 1/2 均为得分姿态且无区别（官方已确认），代码统一用 1；3 为移动姿态
uint8 POSTURE_SCORE = 1
uint8 POSTURE_SCORE_ALT = 2
uint8 POSTURE_MOVE = 3
uint8 posture
