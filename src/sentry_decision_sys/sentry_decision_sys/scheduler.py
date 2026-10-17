"""时间窗任务调度（纯逻辑，不依赖 ROS，便于单测与整局回放）。

输入：比赛时间 t（秒，已换算成「从比赛开始起的秒数」）、当前位姿/速度、当前时刻 now（秒，单调时钟）。
输出：NavCommand 语义的决策（mode / 目标点 / spin / stay）与姿态取值，以及状态文本与预计得分。

阶段（phase）来自 task_table.yaml：
    每个阶段有 [t_start, t_end) 的时间窗、可选前置航点（补给区「离开再进入」用）、目标点、
    停留时长、姿态、小陀螺开关、对应的得分项。
子状态机（每个阶段内）：
    TRAVEL_PRE -> TRAVEL -> HOLD（-> 完成）
    * TRAVEL_PRE：先到前置航点（如补给拉出点）；
    * TRAVEL：到目标点；
    * HOLD：在目标点保持（到位判据：进入半径并连续保持 arrival_hold_sec）；
      停留期间若被推出半径×stay_reset_factor，则停留计时清零（对应「地堡斜坡清空计时」）。
超时：阶段超过 t_end 仍未完成 -> 放弃该阶段（不记分），跳到下一个未过期的阶段。
"""
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

MODE_STOP = 0
MODE_GOTO = 1
MODE_HOLD = 2

POSTURE_SCORE = 1
POSTURE_MOVE = 3

TRAVEL_PRE = "TRAVEL_PRE"   # 去前置航点（补给拉出点）
WAIT = "WAIT"               # 在前置航点等分钟窗口开启（① 的分钟同步）
TRAVEL = "TRAVEL"           # 去目标点
HOLD = "HOLD"               # 在目标点停留


@dataclass
class Point:
    name: str
    x: float = float("nan")
    y: float = float("nan")
    yaw: Optional[float] = None      # None/NaN = 不约束朝向
    radius: float = 0.15

    @property
    def configured(self) -> bool:
        return not (math.isnan(self.x) or math.isnan(self.y))

    def distance(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)


@dataclass
class Phase:
    name: str
    t_start: float
    t_end: float
    point: str
    pre_point: Optional[str] = None
    stay_sec: float = 0.0
    posture: str = "move"            # move | score
    spin_travel: bool = False
    spin_hold: bool = False
    counts_as: Optional[str] = None  # 得分项 key（对应 score 表）；None 表示不计分
    task_id: int = 0
    pre_wait_minute: bool = False    # 在前置航点等到「分钟窗口」开启（末段策略用）

    def task_id_of(self, mapping: Dict[str, int]) -> int:
        return self.task_id or int(mapping.get(self.counts_as or "", 0))


@dataclass
class Decision:
    mode: int = MODE_STOP
    task_id: int = 0
    x: float = 0.0
    y: float = 0.0
    yaw: float = float("nan")
    spin_rate: float = 0.0
    stay_sec: float = 0.0
    radius: float = float("nan")   # 该点位的到位半径 m（导航侧据此判定/驻留，NaN=用导航默认值）
    posture: int = POSTURE_MOVE


@dataclass
class SchedulerConfig:
    leave_supply_dist: float = 6.0        # ① 每分钟「离开再进入」的最小距离
    arrival_hold_sec: float = 0.3         # 到位判据：连续保持在半径内的时间
    stay_reset_factor: float = 1.5        # 停留期间超出 radius*factor 则计时清零
    posture_switch_dist: float = 0.6      # 距目标点小于该距离时切到得分姿态
    posture_min_switch_interval: float = 5.0   # 姿态切换冷却（规则 5 s）
    max_posture_sec: float = 180.0        # 单一姿态累计上限（规则 3 分钟，超出后效果降级）
    spin_rate: float = 360.0              # 小陀螺转速 deg/s
    game_length_sec: float = 300.0
    # 发车门控：必须看到「裁判钟在走」（t 在增加）才认为比赛真的开始了。
    # 准备阶段若裁判已广播 game_time=420（t=0 但不动），绝不能因此发车。
    arm_requires_counting: bool = True
    expand_enabled: bool = True           # 末段策略：按分钟同步循环刷 ①
    minute_entry_window: float = 3.0      # 「刚过整分钟」的判定窗口（秒）
    deadline_margin_sec: float = 12.0     # 等待期间距截止时间不足则直接进入
    score: Dict[str, float] = field(default_factory=dict)
    supply_point: str = "supply"
    pullout_point: str = "supply_pullout"
    task_ids: Dict[str, int] = field(default_factory=dict)


class TaskScheduler:
    def __init__(self, table: Dict[str, Any], config: SchedulerConfig) -> None:
        self.config = config
        self.points: Dict[str, Point] = {}
        for name, raw in (table.get("points") or {}).items():
            raw = raw or {}
            self.points[name] = Point(
                name=name,
                x=float(raw.get("x", float("nan"))),
                y=float(raw.get("y", float("nan"))),
                yaw=(None if raw.get("yaw_deg") is None else
                     (float("nan") if _is_nan(raw.get("yaw_deg")) else math.radians(float(raw["yaw_deg"])))),
                radius=float(raw.get("radius", 0.15)),
            )

        self.phases: List[Phase] = []
        for raw in (table.get("phases") or []):
            self.phases.append(Phase(
                name=str(raw.get("name", "phase")),
                t_start=float(raw.get("t_start", 0.0)),
                t_end=float(raw.get("t_end", self.config.game_length_sec)),
                point=str(raw.get("point", "")),
                pre_point=(None if raw.get("pre_point") in (None, "") else str(raw["pre_point"])),
                stay_sec=float(raw.get("stay_sec", 0.0)),
                posture=str(raw.get("posture", "move")),
                spin_travel=bool(raw.get("spin_travel", False)),
                spin_hold=bool(raw.get("spin_hold", False)),
                counts_as=(None if raw.get("counts_as") in (None, "") else str(raw["counts_as"])),
                task_id=int(raw.get("task_id", 0)),
                pre_wait_minute=bool(raw.get("pre_wait_minute", False)),
            ))

        # 运行状态
        self.phase: Optional[Phase] = None
        self.phase_finished: bool = True
        self.sub_state: str = TRAVEL_PRE
        self.arrived_since: Optional[float] = None
        self.hold_elapsed: float = 0.0
        self.last_update: Optional[float] = None
        self.posture: int = POSTURE_MOVE
        self.posture_changed_at: Optional[float] = None
        self.posture_time: Dict[int, float] = {POSTURE_SCORE: 0.0, POSTURE_MOVE: 0.0}
        self.posture_degraded_logged: bool = False
        self.awards: Dict[str, int] = {}          # 得分项 -> 次数
        self.expected_score: float = 0.0
        self.notes: List[str] = []                # 最近的事件（供日志）
        self.max_dist_supply: float = 0.0         # 本分钟窗口内离补给点的最远距离
        self.last_supply_minute: int = -1          # 已计分过的分钟序号（规则：① 每分钟只算一次）
        self.last_pose: Optional[Tuple[float, float, float]] = None
        self.last_t: Optional[float] = None       # 上一 tick 的 t（判「时钟在走」）
        self.clock_counting: bool = False         # 是否已观察到比赛钟在走（发车门控）
        self.arm_note_printed: bool = False
        self.expand_active: bool = False
        self.expand_cycle: int = 0

    # ------------------------------------------------------------------ 主入口
    def update(self, t: Optional[float], pose: Optional[Tuple[float, float, float]],
               now: float) -> Decision:
        """t=比赛开始后的秒数（未知为 None）；pose=(x, y, yaw)；now=单调秒。"""
        dt = 0.0 if self.last_update is None else max(0.0, now - self.last_update)
        self.last_update = now

        if t is None or pose is None:
            self.posture_time[self.posture] += dt
            return Decision(mode=MODE_STOP, posture=self.posture)

        # 发车门控：准备阶段裁判可能已经在广播 game_time=420（t=0 但钟没走），此时必须待机。
        # 只有观察到 t 在增加（钟真的在走）才 arm；arm 之后不再撤销（比赛中途裁判帧抖动
        # 不应让车反复停走；串口长时间中断另有故障降级）。
        if self.last_t is not None and t > self.last_t + 1e-6:
            self.clock_counting = True
        self.last_t = t
        if self.config.arm_requires_counting and not self.clock_counting:
            self.posture_time[self.posture] += dt
            if not self.arm_note_printed:
                self.arm_note_printed = True
                self._note("等待发车：裁判钟还没开始走（准备阶段）—— 保持零速 + 移动姿态")
            return Decision(mode=MODE_STOP, posture=self.posture)

        self.last_pose = pose
        x, y, yaw = pose

        # ① 的「离开距离」自测：只统计补给点已配置的情况
        supply = self.points.get(self.config.supply_point)
        if supply is not None and supply.configured:
            self.max_dist_supply = max(self.max_dist_supply, supply.distance(x, y))

        self._advance_phase_if_needed(t)
        if self.phase is None or self.phase_finished:
            self._update_posture_time(dt)
            return Decision(mode=MODE_STOP, posture=self.posture)

        decision = self._run_phase(t, x, y, yaw, dt)
        self._update_posture_time(dt)
        return decision

    # ------------------------------------------------------------------ 阶段管理
    def _advance_phase_if_needed(self, t: float) -> None:
        if self.phase is not None and not self.phase_finished:
            return
        if self.phase is not None and self.phase_finished:
            self.phase = None

        while self.phases:
            candidate = self.phases[0]
            if t < candidate.t_start:
                return                      # 还没到第一个阶段的时间窗 -> 待机
            self.phases.pop(0)
            reason = self._phase_skippable(candidate, t)
            if reason is None:
                self._enter_phase(candidate)
                return
            self._note(f"skip {candidate.name}: {reason}")
        # 阶段表跑完 -> 末段策略
        if self.config.expand_enabled:
            self._enter_expand_phase(t)

    def _phase_skippable(self, phase: Phase, t: float) -> Optional[str]:
        if t >= phase.t_end:
            return f"时间窗已过 (t={t:.1f} >= {phase.t_end:.1f})"
        point = self.points.get(phase.point)
        if point is None:
            return f"目标点 {phase.point} 不在 points 表里"
        if not point.configured:
            return f"目标点 {phase.point} 未配置（待打点）"
        if phase.pre_point is not None:
            pre = self.points.get(phase.pre_point)
            if pre is None or not pre.configured:
                return f"前置航点 {phase.pre_point} 未配置（待打点）"
        for name in (phase.point, phase.pre_point):
            if name is None:
                continue
            if not self.points[name].configured:
                return f"航点 {name} 未配置"
        return None

    def _enter_phase(self, phase: Phase) -> None:
        self.phase = phase
        self.phase_finished = False
        self.sub_state = TRAVEL_PRE if phase.pre_point else TRAVEL
        self.arrived_since = None
        self.hold_elapsed = 0.0
        if phase.counts_as == self.config.supply_point or phase.point == self.config.supply_point:
            self.max_dist_supply = 0.0      # 重新开始本分钟窗口的「离开距离」统计
        self._note(f"enter {phase.name} ({self.sub_state})")

    def _enter_expand_phase(self, t: float) -> None:
        """末段策略：补给拉出点等分钟窗口 -> 回补给区停留 5 s，循环刷 ①。

        v1.4 把平分判定从「耗能多者胜」改成了「获得最高得分耗时最短者胜」，所以末段
        **不再为了多耗能而刻意机动**：继续刷 ① 只为「先把总分做高」（第一条判定），
        循环里等待整分钟是规则要求（每分钟必须先离开再进入）；姿态用移动姿态是为了
        拿到 ×1.5 的功率上限、跑得更快（时间效率）。
        """
        pullout = self.points.get(self.config.pullout_point)
        if pullout is None or not pullout.configured:
            self._note("expand skipped: 拉出点未配置")
            return
        self.expand_active = True
        self.expand_cycle += 1
        # 周期由「分钟同步」给出：在拉出点等到下一个整分钟再进补给区（对应规则「每分钟一次」）
        self._enter_phase(Phase(
            name=f"expand_margin_{self.expand_cycle}",
            t_start=0.0,
            t_end=self.config.game_length_sec,   # 不许跑过比赛时长（否则最后几秒还在往补给区跑）
            point=self.config.supply_point,
            pre_point=self.config.pullout_point,
            stay_sec=5.0,
            posture="move",
            counts_as=self.config.supply_point,
            pre_wait_minute=True,
        ))

    # ------------------------------------------------------------------ 阶段执行
    def _run_phase(self, t: float, x: float, y: float, yaw: float,
                   dt: float) -> Decision:
        phase = self.phase
        assert phase is not None

        if t > phase.t_end:
            self._note(f"abort {phase.name}: 超时未完成 (t={t:.1f} > {phase.t_end:.1f})")
            self._finish_phase(award=False, t=t)
            return Decision(mode=MODE_STOP, posture=self._desired_posture(t, x, y))

        target_name = phase.point if self.sub_state in (TRAVEL, HOLD) else phase.pre_point
        if target_name is None:
            self._note(f"abort {phase.name}: 目标点缺失")
            self._finish_phase(award=False, t=t)
            return Decision(mode=MODE_STOP, posture=self._desired_posture(t, x, y))
        target = self.points[target_name]

        distance = target.distance(x, y)
        arrived = distance <= target.radius

        if self.sub_state in (TRAVEL_PRE, TRAVEL):
            if arrived:
                if self.arrived_since is None:
                    self.arrived_since = t
                if t - self.arrived_since >= self.config.arrival_hold_sec:
                    self._on_arrive(t, phase, target_name)
            else:
                self.arrived_since = None

        if self.sub_state == HOLD:
            if distance > target.radius * self.config.stay_reset_factor:
                if self.hold_elapsed > 0.0:
                    self._note(f"{phase.name}: 离开停留半径，停留计时清零 "
                               f"(d={distance:.2f} > {target.radius * self.config.stay_reset_factor:.2f})")
                self.hold_elapsed = 0.0
            else:
                self.hold_elapsed += dt

            if self.hold_elapsed >= phase.stay_sec:
                self._note(f"{phase.name}: 停留完成 {self.hold_elapsed:.1f}s >= {phase.stay_sec:.1f}s")
                self._finish_phase(award=True, t=t)

        # 分钟同步：在前置航点等到「刚过整分钟」再进入目标区，保证停留落在该分钟内
        if self.sub_state == WAIT:
            minute_ok = (t % 60.0) <= self.config.minute_entry_window
            deadline_near = t >= phase.t_end - (phase.stay_sec + self.config.deadline_margin_sec)
            if minute_ok or deadline_near:
                self._note(f"{phase.name}: 分钟窗口开启"
                           f"{'（截止临近，直接进入）' if not minute_ok else ''}，前往 {phase.point}")
                self.sub_state = TRAVEL
                self.arrived_since = None

        # 本 tick 内阶段已结束（到达即完成 / 停留完成 / 超时）：先回待机，下一 tick 再进新阶段
        if self.phase_finished:
            return Decision(mode=MODE_STOP, posture=self.posture)

        posture = self._desired_posture(t, x, y)
        if self.sub_state in (HOLD, WAIT):
            return Decision(mode=MODE_HOLD, task_id=phase.task_id_of(self.config.task_ids),
                            x=target.x, y=target.y, yaw=_yaw_of(target),
                            spin_rate=(self.config.spin_rate if phase.spin_hold else 0.0),
                            stay_sec=phase.stay_sec, radius=target.radius, posture=posture)
        return Decision(mode=MODE_GOTO, task_id=phase.task_id_of(self.config.task_ids),
                        x=target.x, y=target.y, yaw=_yaw_of(target),
                        spin_rate=(self.config.spin_rate if phase.spin_travel else 0.0),
                        stay_sec=phase.stay_sec, radius=target.radius, posture=posture)

    def _on_arrive(self, t: float, phase: Phase, target_name: str) -> None:
        self.arrived_since = None
        if self.sub_state == TRAVEL_PRE:
            if phase.pre_wait_minute:
                self._note(f"{phase.name}: 到达前置航点 {target_name}，等分钟窗口（t={t:.1f}）")
                self.sub_state = WAIT
            else:
                self._note(f"{phase.name}: 到达前置航点 {target_name}")
                self.sub_state = TRAVEL
            return
        if phase.stay_sec > 0.0 or phase.pre_wait_minute:
            self._note(f"{phase.name}: 到达 {target_name}，开始停留")
            self.sub_state = HOLD
            self.hold_elapsed = 0.0
        else:
            self._note(f"{phase.name}: 到达 {target_name}（无需停留）")
            self._finish_phase(award=True, t=t)

    def _finish_phase(self, award: bool, t: float) -> None:
        phase = self.phase
        assert phase is not None
        if award and phase.counts_as:
            self._award(phase, t)
        self.phase_finished = True

    # ------------------------------------------------------------------ 得分记账
    def _award(self, phase: Phase, t: float) -> None:
        item = phase.counts_as
        config = self.config
        if t > config.game_length_sec + 0.5:
            # 裁判钟已过比赛时长（420→120 走完）；之后即使阶段还在跑也不再记分（0.5 s 容差留给帧量化）
            self._note(f"不计分：t={t:.1f}s 已超过比赛时长 {config.game_length_sec:.0f}s")
            return
        if item == config.supply_point:
            if self.max_dist_supply < config.leave_supply_dist:
                self._note(f"①无效：本分钟最远离补给点 {self.max_dist_supply:.2f} m "
                           f"< {config.leave_supply_dist:.2f} m，不计分")
                return
            minute = int(t // 60.0)
            if minute == self.last_supply_minute:
                # 规则：每分钟只算一次；阶段表里多留的 ① 阶段遇到时间漂移时不应重复计分
                self._note(f"①无效：第 {minute + 1} 分钟已经计过分（规则每分钟一次）")
                return
            self.last_supply_minute = minute
            self._note(f"①有效（第 {minute + 1} 分钟）：最远离补给点 {self.max_dist_supply:.2f} m")
        points = float(config.score.get(item or "", 0.0))
        self.awards[item] = self.awards.get(item, 0) + 1
        self.expected_score += points
        self._note(f"得分 {item} +{points:g}（累计 {self.expected_score:g}）")
        # ⑤：从补给区出发、全程开小陀螺到达加分停留点（乘倍数已确认，per-trip 计一次）
        if phase.spin_travel and item in ("central_high", "trapezoid_high", "bunker"):
            bonus = float(config.score.get("spin_bonus", 0.0))
            self.awards["spin_bonus"] = self.awards.get("spin_bonus", 0) + 1
            self.expected_score += bonus
            self._note(f"⑤ 小陀螺加分 +{bonus:g}（累计 {self.expected_score:g}）")

    # ------------------------------------------------------------------ 姿态
    def _desired_posture(self, t: float, x: float, y: float) -> int:
        phase = self.phase
        want_score = False
        if phase is not None:
            if self.sub_state == HOLD:
                want_score = phase.posture == "score"
            elif phase.posture == "score":
                target = self.points.get(phase.point)
                if target is not None and target.configured:
                    want_score = target.distance(x, y) <= self.config.posture_switch_dist
        desired = POSTURE_SCORE if want_score else POSTURE_MOVE

        if desired != self.posture:
            if (self.posture_changed_at is None or
                    t - self.posture_changed_at >= self.config.posture_min_switch_interval):
                self.posture = desired
                self.posture_changed_at = t
                self._note(f"姿态切换 -> {desired}（1=得分 3=移动）@t={t:.1f}")
            # 冷却未到则维持当前姿态
        return self.posture

    def _update_posture_time(self, dt: float) -> None:
        self.posture_time[self.posture] = self.posture_time.get(self.posture, 0.0) + dt
        if (not self.posture_degraded_logged and
                self.posture_time[self.posture] > self.config.max_posture_sec):
            self.posture_degraded_logged = True
            self._note(f"警告：当前姿态累计 {self.posture_time[self.posture]:.0f}s "
                       f"超过 {self.config.max_posture_sec:.0f}s，效果将降级（×1.2）")

    # ------------------------------------------------------------------ 日志
    def _note(self, text: str) -> None:
        self.notes.append(text)
        if len(self.notes) > 32:
            del self.notes[:len(self.notes) - 32]

    def force_posture(self, posture: int) -> None:
        """故障降级：强制切到指定姿态（故障优先，不走 5 s 冷却；数据恢复后由正常流程接管）。"""
        if self.posture != posture:
            self._note(f"故障降级：姿态强制 -> {posture}")
        self.posture = posture
        self.posture_changed_at = self.last_update

    def pop_notes(self) -> List[str]:
        notes, self.notes = self.notes, []
        return notes

    def status_text(self) -> str:
        phase = self.phase.name if self.phase is not None else "none"
        return (f"phase={phase} sub={self.sub_state} posture={self.posture} "
                f"hold={self.hold_elapsed:.1f}s score={self.expected_score:g} "
                f"awards={self.awards} posture_time={self.posture_time}")


def _is_nan(value: Any) -> bool:
    try:
        return math.isnan(float(value))
    except (TypeError, ValueError):
        return True


def _yaw_of(point: Point) -> float:
    return float("nan") if point.yaw is None else point.yaw
