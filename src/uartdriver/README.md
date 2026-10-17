# UART串口通信驱动

## 使用教程
1. 正确地打开串口
- 在命令行输入 `ls /dev/tty*`查找串口对应的文件，例如找到了**dev/ttyUSB0**

- 命令行输入`sudo udevadm info --query=all --name=/dev/ttyUSB0`查找串口文件的详细信息

- 找到**ID_SERIAL_SHORT=c6371f124814eb119da4437c2d17fe54**信息所在位置，将后面的序列号复制保存到config文件里面

## 数据链路

### 接收（电控 → ROS2）

| 串口包 id | 内容 | 发布话题 |
|---|---|---|
| `0x11` | 裁判系统（`game_time`、`sentry_posture`） | `/referee_data`（`rm_msgs/RefereeData`） |
| `0x16` | 底盘数据（`vx`、`vy`、`wz`） | `/chassis_data`（`rm_msgs/ChassisData`） |

接收侧会校验**帧长 + 帧头 + id + 帧尾 + CRC16**，任一项不符即**丢弃该帧并计数**（1 s 一次节流告警，日志里能看到累计丢弃数）；
不会把校验失败的帧变成 `game_time=0` 之类的**假数据**发布出去。

### 发送（ROS2 → 电控）

| 订阅话题 | 消息 | 串口包 id |
|---|---|---|
| `/sentry/velocity_command` | `rm_msgs/VelocityCommand`：`vx/vy` mm/s（车体系）、`wz` deg/s（逆时针为正） | `0x20` |
| `/sentry/posture_command` | `rm_msgs/PostureCommand`：姿态（1=得分姿态，2 等价；3=移动姿态） | `0x20` |

发送由 `send_rate_hz` 定时器周期完成（默认 50 Hz）：把最新的速度与姿态填入 `ChassisControl`
（`ph1` 固定填 **4**（电控 2026-10-17 要求），其余占位字节置 0），再按 `0x7D | id | payload | CRC16 | 0x7E` 组帧，帧内 `0x7D/0x7E/0x7F` 做转义后写入串口。

两个看门狗（都在发送定时器里判定）：

- `velocity_timeout`（默认 0.2 s）：超时未收到速度指令 → 发零速，绝不沿用旧指令；
- `posture_timeout`（默认 1.0 s）：超时未收到姿态指令 → 回到 `default_posture`（移动姿态），避免决策掉线后卡在得分姿态。

串口打开失败不再让节点退出：读线程每 2 s 重连一次（无硬件时节点照常启动、话题照常建立，方便检录）。
发送线程的 `write` 与读线程的 `close/open` 共用一把端口互斥锁，重连不会打断正在进行的写或反之；重连成功后立刻恢复发送。

## 参数（config/driver_config.yaml）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `serial_name` | `/dev/ttyACM0` | 串口设备名（`serial_number` 为空白时使用） |
| `serial_number` | `" "` | USB 序列号，非空白时优先按序列号匹配设备 |
| `baud_rate` | `1000000` | 波特率 |
| `read_buffer_size` | `60` | 单帧读缓冲（小于 32 会被强制改为 60） |
| `possible_ids` | `[17, 22]` | 允许的接收包 id（十进制：0x11、0x16） |
| `velocity_command_topic` | `/sentry/velocity_command` | 速度指令话题 |
| `posture_command_topic` | `/sentry/posture_command` | 姿态指令话题 |
| `send_rate_hz` | `50.0` | `0x20` 包发送频率 |
| `velocity_timeout` | `0.2` | 速度指令超时（s） |
| `posture_timeout` | `1.0` | 姿态指令超时（s） |
| `default_posture` | `3` | 未收到/超时姿态时使用的姿态（3 移动姿态） |

节点启动时会打印全部参数（`[param] ...`），便于确认 yaml 是否传递成功。
