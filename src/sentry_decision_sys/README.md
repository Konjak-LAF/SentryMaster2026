# sentry_decision_sys —— 决策节点

时间窗任务调度：按「比赛时间轴 + 任务表」决定**去哪个点、停多久、用哪种姿态、要不要开小陀螺**，
周期发布 `/sentry/nav_command`（任务级指令）与 `/sentry/posture_command`（姿态）。

## 模块划分

| 文件 | 职责 |
|---|---|
| `sentry_decision_sys/scheduler.py` | **纯逻辑**状态机（不依赖 ROS，可用虚拟时钟跑完整局做仿真）：阶段序列、① 的分钟同步「离开再进入」、到位/停留判据（离开半径则计时清零）、姿态编排（5 s 冷却 + 180 s 预算）、小陀螺、末段循环、得分记账 |
| `sentry_decision_sys/decision_node.py` | ROS 胶水：参数、任务表加载、`game_time` → `t` 换算、tf 位姿、话题收发、日志 |
| `config/decision.yaml` | **引擎旋钮**（超时、判据、姿态、限速相关、末段开关） |
| `config/task_table.yaml` | **比赛数据**（点位坐标、阶段时间表、得分表）——点位需自行标定 |

## 时间轴

官方口径（`rulev1.4` §6.6）：比赛开始 `game_time = 420`，每秒减 1，比赛结束减到 `120`，
裁判终止后归 `0`。因此：

```
t = game_time_at_start - game_time        # 420 -> 120 对应 t = 0 -> 300 s
game_time == 0  =>  未开始 / 已终止  =>  待机（不发运动指令）
```

`game_time_at_start` / `game_time_at_end` 都是参数（默认 420/120）。若电控改成递增计时，
节点会按「相邻帧增量」自动判定方向并退化为 `t = game_time`。

## 话题

| 方向 | 话题 | 消息 | 说明 |
|---|---|---|---|
| 订阅 | `/referee_data` | `rm_msgs/RefereeData` | `game_time`（420→120 递减）、`sentry_posture` 回传 |
| 订阅 | `/odin1/odometry_highfreq` | `nav_msgs/Odometry` | 只用于状态日志里的速度；位姿走 tf |
| 订阅 | `/chassis_data` | `rm_msgs/ChassisData` | 底盘回传速度（状态日志） |
| 订阅 | tf `odom → vehicle` | — | 自身位姿（打点与到位判定都用这个坐标系） |
| 发布 | `/sentry/nav_command` | `rm_msgs/NavCommand` | `mode`(0 停 / 1 前往 / 2 驻留)、目标点、`spin_rate`、`stay_sec` |
| 发布 | `/sentry/posture_command` | `rm_msgs/PostureCommand` | 1=得分姿态、3=移动姿态（官方确认 1/2 等价） |

## 安全兜底

* 裁判数据超时 `referee_timeout`（0.5 s）或位姿超时 `pose_timeout`（0.5 s）→ 待机零速；
* 点位未标定（`task_table.yaml` 里 `x/y` 为 `NaN`）→ 对应阶段自动跳过并告警，**不会开去 (0,0)**；
* 阶段超过 `t_end` → 放弃该阶段（不记分）并跳到下一个未过期阶段；
* 姿态切换遵守 5 s 冷却；单一姿态累计超 180 s 会打降级告警（规则：效果 ×1.2）。

## 用法

```bash
ros2 launch sentry_decision_sys launch.py                      # 用本包默认参数与任务表
ros2 launch sentry_decision_sys launch.py params_file:=<yaml>  # 覆盖参数文件
ros2 run sentry_decision_sys sentry_decision_node --ros-args \
  --params-file install/sentry_decision_sys/share/sentry_decision_sys/config/decision.yaml \
  -p task_table_file:=<task_table.yaml>                        # 换一张任务表
```

启动时会打印全部参数（`[param] ...`）与任务表摘要（已配置/待打点/阶段数/得分表），
运行中 1 Hz 打印 `[status]`，阶段切换、到位、停留清零、得分、姿态切换等事件用 `[fsm]` 打出。
