# 导航 -> uart_driver：底盘速度指令，单位与 ChassisData 一致（2.md §3）
std_msgs/Header header

float32 vx   # mm/s，车体系正前方
float32 vy   # mm/s，车体系左方
float32 wz   # deg/s，逆时针为正；即底盘自旋转速（小陀螺与转向共用）
