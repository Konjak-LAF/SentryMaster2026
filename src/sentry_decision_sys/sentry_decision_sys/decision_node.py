"""决策节点。

阶段说明（见 2.md）：
  * P0（当前）：只打通指令链路与参数化——以“待机”语义周期发布
    NavCommand(MODE_STOP) 与 PostureCommand(移动姿态)，并 1 Hz 打印裁判/里程计状态，
    便于确认话题连通与 yaml 参数传递。
  * P2：时间窗调度（补给区/中央高地/梯形高地/地堡的时序、姿态编排、超时降级）在此节点实装。
"""
import math

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from nav_msgs.msg import Odometry
from rm_msgs.msg import ChassisData, NavCommand, PostureCommand, RefereeData


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
        "posture_score": 1,
        "posture_move": 3,
        "standby_posture": 3,
        "standby_nav_mode": NavCommand.MODE_STOP,
        "standby_task_id": NavCommand.TASK_IDLE,
    }

    def __init__(self) -> None:
        super().__init__("sentry_decision_node")

        for name, value in self.DEFAULT_PARAMETERS.items():
            self.declare_parameter(name, value)

        # 打印本次启动实际生效的参数（预检录要求：确认 yaml 是否传递成功）
        for name in self.DEFAULT_PARAMETERS:
            self.get_logger().info(f"[param] {name} = {self.get_parameter(name).value}")

        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.command_rate_hz = self._positive_rate("command_rate_hz")
        self.status_rate_hz = self._positive_rate("status_rate_hz")
        self.standby_nav_mode = self.get_parameter("standby_nav_mode").value
        self.standby_task_id = self.get_parameter("standby_task_id").value
        self.standby_posture = self.get_parameter("standby_posture").value

        # 最近一次收到的数据
        self.game_time = None
        self.posture = None
        self.odom = None
        self.chassis = None
        self.referee_count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_puber = self.create_publisher(
            msg_type=NavCommand,
            topic=self.get_parameter("nav_command_topic").value,
            qos_profile=10,
        )
        self.posture_puber = self.create_publisher(
            msg_type=PostureCommand,
            topic=self.get_parameter("posture_command_topic").value,
            qos_profile=10,
        )

        self.referee_suber = self.create_subscription(
            msg_type=RefereeData,
            topic=self.get_parameter("referee_topic").value,
            callback=self.referee_callback,
            qos_profile=10,
        )

        self.odom_suber = self.create_subscription(
            msg_type=Odometry,
            topic=self.get_parameter("odom_topic").value,
            callback=self.odom_callback,
            qos_profile=10,
        )

        self.chassis_suber = self.create_subscription(
            msg_type=ChassisData,
            topic=self.get_parameter("chassis_data_topic").value,
            callback=self.chassis_callback,
            qos_profile=10,
        )

        self.command_timer = self.create_timer(1.0 / self.command_rate_hz, self.publish_commands)
        self.status_timer = self.create_timer(1.0 / self.status_rate_hz, self.log_status)

        self.get_logger().info("sentry_decision_node has been built")

    def _positive_rate(self, name: str) -> float:
        rate = float(self.get_parameter(name).value)
        if rate <= 0.0:
            self.get_logger().error(f"{name}={rate} is invalid, use 10.0 instead")
            return 10.0
        return rate

    # ------------------------------------------------------------------ 指令发布
    def publish_commands(self) -> None:
        """发布待机指令：导航急停 + 移动姿态。P2 会用时间窗调度替换本函数。"""
        stamp = self.get_clock().now().to_msg()

        nav = NavCommand()
        nav.header.stamp = stamp
        nav.header.frame_id = self.global_frame
        nav.mode = self.standby_nav_mode
        nav.task_id = self.standby_task_id
        # MODE_STOP 下目标点无意义，置 0；yaw 用 NaN 表示不约束朝向
        nav.x = 0.0
        nav.y = 0.0
        nav.yaw = float("nan")
        nav.spin_rate = 0.0
        nav.stay_sec = 0.0
        self.nav_puber.publish(nav)

        posture = PostureCommand()
        posture.header.stamp = stamp
        posture.posture = self.standby_posture
        self.posture_puber.publish(posture)

    # ------------------------------------------------------------------ 数据回调
    def referee_callback(self, msg: RefereeData) -> None:
        self.game_time = msg.game_time
        self.posture = msg.sentry_posture
        self.referee_count += 1

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def chassis_callback(self, msg: ChassisData) -> None:
        self.chassis = msg

    # ------------------------------------------------------------------ 状态打印
    def lookup_pose(self):
        """查 global_frame -> robot_frame 的位姿，失败返回 None。"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame, rclpy.time.Time()
            )
        except Exception as ex:  # tf2 在链路未建立时抛 TransformException
            self.get_logger().warning(f"tf {self.global_frame}->{self.robot_frame} unavailable: {ex}",
                                      throttle_duration_sec=5.0)
            return None

        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
        )
        return (translation.x, translation.y, yaw)

    def log_status(self) -> None:
        pose = self.lookup_pose()
        pose_text = "n/a" if pose is None else f"({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.1f}deg)"
        velocity_text = "n/a"
        if self.odom is not None:
            twist = self.odom.twist.twist.linear
            velocity_text = f"({twist.x:.2f}, {twist.y:.2f}) m/s"
        self.get_logger().info(
            f"[status] game_time={self.game_time} posture={self.posture} "
            f"pose={pose_text} odom_v={velocity_text} referee_frames={self.referee_count}"
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
