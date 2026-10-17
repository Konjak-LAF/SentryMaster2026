"""决策节点：时间窗任务调度（P2）。

职责：
  * 订阅 `/referee_data`（game_time / sentry_posture）、`/odin1/odometry_highfreq`、`/chassis_data`；
  * 用 tf(`odom -> vehicle`) 查自身位姿；
  * 用 `scheduler.TaskScheduler` 按「任务表（点位 + 阶段时间表 + 得分表）」决定：
    现在去哪个点、要不要停留/停留多久、要不要开小陀螺、用哪种姿态；
  * 周期发布 `/sentry/nav_command`（任务级指令）与 `/sentry/posture_command`（姿态），
    并把「当前阶段 / 预计得分 / 事件」打到日志里。

阶段与点位等**数据**在 `config/task_table.yaml`（待打点实测填入），
本文件的参数只放引擎旋钮（超时、判据、姿态切换等），见 `config/decision.yaml`。
"""
import math
import os

import rclpy
import yaml
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from nav_msgs.msg import Odometry
from rm_msgs.msg import ChassisData, NavCommand, PostureCommand, RefereeData

from sentry_decision_sys.scheduler import POSTURE_MOVE, SchedulerConfig, TaskScheduler


class DecisionNode(Node):
    # 参数名 -> 默认值（yaml 覆盖，见 config/decision.yaml）
    DEFAULT_PARAMETERS = {
        "nav_command_topic": "/sentry/nav_command",
        "posture_command_topic": "/sentry/posture_command",
        "referee_topic": "/referee_data",
        "odom_topic": "/odin1/odometry_highfreq",
        "chassis_data_topic": "/chassis_data",
        "global_frame": "odom",
        "robot_frame": "vehicle",
        "command_rate_hz": 10.0,
        "status_rate_hz": 1.0,
        # 任务表（点位/阶段/得分）
        "task_table_file": "",
        # 时间轴与数据有效性
        "game_length_sec": 300.0,
        # 发车门控：必须看到裁判钟在走（t 增加）才执行阶段表；准备阶段广播的静态 420 不发车
        "arm_requires_counting": True,
        # 官方口径（v1.4 §6.6）：比赛开始 game_time=420，每秒减 1，结束时减到 120，
        # 裁判终止后归 0 —— 所以 t = game_time_at_start - game_time
        "game_time_at_start": 420.0,
        "game_time_at_end": 120.0,
        "referee_timeout": 0.5,
        "pose_timeout": 0.5,
        # 串口(裁判/底盘数据)故障降级：中断超过该时长 -> 停车 + 回移动姿态（省电、安全）
        "serial_fault_timeout": 2.0,
        # ①「离开再进入」的最小距离（官方称阈值后续见规则修改，这里留余量）
        "leave_supply_dist": 6.0,
        # 到位/停留判据
        "arrival_hold_sec": 0.3,
        "stay_reset_factor": 1.5,
        # 姿态
        "posture_switch_dist": 0.6,
        "posture_min_switch_interval": 5.0,
        "max_posture_sec": 180.0,
        "spin_rate": 360.0,
        # 末段策略（扩大获胜面：按分钟同步循环刷 ①，走移动姿态以多耗能）
        "expand_enabled": True,
        "minute_entry_window": 3.0,
        "deadline_margin_sec": 12.0,
    }

    def __init__(self) -> None:
        super().__init__("sentry_decision_node")

        for name, value in self.DEFAULT_PARAMETERS.items():
            self.declare_parameter(name, value)

        # 打印本次启动实际生效的参数（预检录要求：确认 yaml 是否传递成功）
        for name in self.DEFAULT_PARAMETERS:
            self.get_logger().info(f"[param] {name} = {self.get_parameter(name).value}")

        param = self.get_parameter
        self.global_frame = param("global_frame").value
        self.robot_frame = param("robot_frame").value
        self.command_rate_hz = self._positive(param("command_rate_hz").value, "command_rate_hz")
        self.status_rate_hz = self._positive(param("status_rate_hz").value, "status_rate_hz")
        self.game_length_sec = float(param("game_length_sec").value)
        self.game_time_at_start = float(param("game_time_at_start").value)
        self.game_time_at_end = float(param("game_time_at_end").value)
        self.referee_timeout = float(param("referee_timeout").value)
        self.pose_timeout = float(param("pose_timeout").value)
        self.serial_fault_timeout = float(param("serial_fault_timeout").value)

        # 最近一次收到的数据 + 到达时刻
        self.game_time = None
        self.game_time_stamp = None
        self.game_counting_up = None      # game_time 到底是「已进行」还是「剩余」，自动判定
        self.posture_reported = None
        self.odom = None
        self.chassis = None
        self.chassis_stamp = None
        self.pose = None
        self.pose_stamp = None
        self.referee_count = 0

        # 任务表 + 调度器
        self.table = self._load_task_table()
        config = SchedulerConfig(
            leave_supply_dist=float(param("leave_supply_dist").value),
            arrival_hold_sec=float(param("arrival_hold_sec").value),
            stay_reset_factor=float(param("stay_reset_factor").value),
            posture_switch_dist=float(param("posture_switch_dist").value),
            posture_min_switch_interval=float(param("posture_min_switch_interval").value),
            max_posture_sec=float(param("max_posture_sec").value),
            spin_rate=float(param("spin_rate").value),
            game_length_sec=self.game_length_sec,
            arm_requires_counting=bool(param("arm_requires_counting").value),
            expand_enabled=bool(param("expand_enabled").value),
            minute_entry_window=float(param("minute_entry_window").value),
            deadline_margin_sec=float(param("deadline_margin_sec").value),
            score={k: float(v) for k, v in (self.table.get("score") or {}).items()},
            task_ids={k: int(v) for k, v in (self.table.get("task_ids") or {}).items()},
        )
        self.scheduler = TaskScheduler(self.table, config)
        self._log_table_summary()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_puber = self.create_publisher(
            msg_type=NavCommand,
            topic=param("nav_command_topic").value,
            qos_profile=10,
        )
        self.posture_puber = self.create_publisher(
            msg_type=PostureCommand,
            topic=param("posture_command_topic").value,
            qos_profile=10,
        )

        self.referee_suber = self.create_subscription(
            msg_type=RefereeData,
            topic=param("referee_topic").value,
            callback=self.referee_callback,
            qos_profile=10,
        )
        self.odom_suber = self.create_subscription(
            msg_type=Odometry,
            topic=param("odom_topic").value,
            callback=self.odom_callback,
            qos_profile=10,
        )
        self.chassis_suber = self.create_subscription(
            msg_type=ChassisData,
            topic=param("chassis_data_topic").value,
            callback=self.chassis_callback,
            qos_profile=10,
        )

        self.command_timer = self.create_timer(1.0 / self.command_rate_hz, self.publish_commands)
        self.status_timer = self.create_timer(1.0 / self.status_rate_hz, self.log_status)

        self.get_logger().info("sentry_decision_node has been built")

    # ------------------------------------------------------------------ 初始化辅助
    def _positive(self, value, name: str) -> float:
        rate = float(value)
        if rate <= 0.0:
            self.get_logger().error(f"{name}={rate} is invalid, use 10.0 instead")
            return 10.0
        return rate

    def _load_task_table(self) -> dict:
        path = str(self.get_parameter("task_table_file").value)
        if not path:
            try:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(get_package_share_directory("sentry_decision_sys"),
                                    "config", "task_table.yaml")
            except Exception as ex:
                self.get_logger().error(f"cannot locate default task table: {ex}")
                return {}
        try:
            with open(path, encoding="utf-8") as handle:
                table = yaml.safe_load(handle) or {}
            self.get_logger().info(f"[task_table] loaded {path}")
            return table
        except Exception as ex:
            self.get_logger().error(f"failed to load task table {path}: {ex}")
            return {}

    def _log_table_summary(self) -> None:
        points = self.scheduler.points
        configured = [name for name, point in points.items() if point.configured]
        pending = [name for name, point in points.items() if not point.configured]
        self.get_logger().info(
            f"[task_table] points={len(points)} 已配置={configured or '无'} "
            f"待打点={pending or '无'} phases={len(self.scheduler.phases)} "
            f"score_table={self.scheduler.config.score}"
        )
        if pending:
            self.get_logger().warning(
                f"[task_table] {len(pending)} 个点位未配置（NaN），对应阶段会被跳过；"
                f"请先用打点工具填 config/task_table.yaml"
            )

    # ------------------------------------------------------------------ 数据回调
    def referee_callback(self, msg: RefereeData) -> None:
        if self.game_time is not None:
            delta = int(msg.game_time) - int(self.game_time)
            # 只用「小幅变化」判定方向（跨 300/0 的回绕会给出几百的跳变，忽略）
            if 0 < abs(delta) < 100:
                self.game_counting_up = delta > 0
        self.game_time = int(msg.game_time)
        self.game_time_stamp = self.get_clock().now()
        self.posture_reported = msg.sentry_posture
        self.referee_count += 1

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def chassis_callback(self, msg: ChassisData) -> None:
        self.chassis = msg
        self.chassis_stamp = self.get_clock().now()

    # ------------------------------------------------------------------ 时间/位姿
    def elapsed_time(self):
        """把 referee 的 game_time 换算成「比赛开始后的秒数」；无效/不在比赛中返回 None。

        官方口径（rulev1.4 §6.6）：比赛开始 game_time=420，每秒钟减 1，比赛结束减到 120，
        裁判终止比赛后 game_time 归 0 —— 因此：
            t = game_time_at_start - game_time     （本例 420 -> 120 对应 t = 0 -> 300）
            game_time == 0：比赛未开始或已终止 -> 待机（不再驱动）
        """
        if self.game_time is None or self.game_time_stamp is None:
            return None
        age = (self.get_clock().now() - self.game_time_stamp).nanoseconds * 1e-9
        if age > self.referee_timeout:
            return None
        if self.game_time == 0:
            return None      # 未开始 / 已终止
        if self.game_counting_up is False:
            return max(0.0, self.game_time_at_start - float(self.game_time))
        if self.game_counting_up is True:
            return float(self.game_time)
        # 方向还没判出来（第一条裁判帧）：按官方区间判定 —— 落在 [120, 420] 里就是递减口径
        first = float(self.game_time)
        if self.game_time_at_end <= first <= self.game_time_at_start:
            return max(0.0, self.game_time_at_start - first)
        return first

    def lookup_pose(self):
        """查 global_frame -> robot_frame 的位姿，失败返回 None。"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time()
            )
        except Exception as ex:  # tf2 在链路未建立时抛 TransformException
            self.get_logger().warning(f"tf {self.global_frame}->{self.robot_frame} unavailable: {ex}",
                                      throttle_duration_sec=5.0)
            self.pose = None
            self.pose_stamp = None
            return None

        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
        )
        self.pose = (translation.x, translation.y, yaw)
        self.pose_stamp = self.get_clock().now()
        return self.pose

    def _pose_is_fresh(self) -> bool:
        if self.pose is None or self.pose_stamp is None:
            return False
        age = (self.get_clock().now() - self.pose_stamp).nanoseconds * 1e-9
        return age <= self.pose_timeout

    def _serial_fault(self) -> bool:
        """串口数据（裁判帧 + 底盘帧）是否长时间中断 —— 触发 P3.7 故障降级。

        串口一断，电控收不到新指令也收不到姿态，只会停在最后一个姿态上；
        因此这里长时间无数据时主动下发「停车 + 移动姿态」，把功率与动作都降下来。
        """
        now = self.get_clock().now()
        for stamp in (self.game_time_stamp, self.chassis_stamp):
            if stamp is None:
                return True
            if (now - stamp).nanoseconds * 1e-9 > self.serial_fault_timeout:
                return True
        return False

    # ------------------------------------------------------------------ 指令发布
    def publish_commands(self) -> None:
        stamp = self.get_clock().now().to_msg()

        if self._serial_fault():
            self.scheduler.force_posture(POSTURE_MOVE)
            nav = NavCommand()
            nav.header.stamp = stamp
            nav.header.frame_id = self.global_frame
            nav.mode = NavCommand.MODE_STOP
            nav.radius = float("nan")
            self.nav_puber.publish(nav)
            posture = PostureCommand()
            posture.header.stamp = stamp
            posture.posture = POSTURE_MOVE
            self.posture_puber.publish(posture)
            self.get_logger().error(
                f"串口数据中断 > {self.serial_fault_timeout:.1f}s：停车 + 回移动姿态",
                throttle_duration_sec=2.0,
            )
            return

        pose = self.lookup_pose()
        if not self._pose_is_fresh():
            pose = None

        decision = self.scheduler.update(
            self.elapsed_time(), pose, self.get_clock().now().nanoseconds * 1e-9
        )

        nav = NavCommand()
        nav.header.stamp = stamp
        nav.header.frame_id = self.global_frame
        nav.mode = decision.mode
        nav.task_id = decision.task_id
        nav.x = float(decision.x)
        nav.y = float(decision.y)
        nav.yaw = float(decision.yaw)
        nav.spin_rate = float(decision.spin_rate)
        nav.stay_sec = float(decision.stay_sec)
        nav.radius = float(decision.radius)
        self.nav_puber.publish(nav)

        posture = PostureCommand()
        posture.header.stamp = stamp
        posture.posture = decision.posture     # 1=得分 3=移动（取值已确认，见 2.md §1.1）
        self.posture_puber.publish(posture)

    # ------------------------------------------------------------------ 状态打印
    def log_status(self) -> None:
        for note in self.scheduler.pop_notes():
            self.get_logger().info(f"[fsm] {note}")

        t = self.elapsed_time()
        t_text = "n/a" if t is None else f"{t:6.1f}s"
        pose_text = "n/a"
        if self.pose is not None:
            pose_text = f"({self.pose[0]:.2f}, {self.pose[1]:.2f}, {math.degrees(self.pose[2]):.0f}deg)"
        velocity_text = "n/a"
        if self.odom is not None:
            twist = self.odom.twist.twist.linear
            velocity_text = f"{math.hypot(twist.x, twist.y):.2f}m/s"
        chassis_text = "n/a"
        if self.chassis is not None:
            chassis_text = f"{math.hypot(self.chassis.vx, self.chassis.vy):.0f}mm/s"
        self.get_logger().info(
            f"[status] t={t_text} pose={pose_text} v={velocity_text} "
            f"chassis_v={chassis_text} "
            f"game_time={self.game_time}(up={self.game_counting_up}) "
            f"referee_posture={self.posture_reported} frames={self.referee_count} | "
            f"{self.scheduler.status_text()}"
        )


def cleanup_node(node: Node) -> None:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def spin_node(node: Node) -> None:
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        spin_node(node)
    finally:
        cleanup_node(node)
