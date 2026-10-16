#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fake_ecu.py —— 无硬件的「假 ECU」：uart_driver 串口链路台架对端

干什么
------
在不接真实电控板的情况下扮演 ECU 一侧，把 `uart_driver_node` 的整条串口链路跑通并自检：

1. **发（ECU -> 驱动）**：按仓库里 `struct_def.hpp` / `serial_trans_tool.hpp` 的定义，发送
   `0x11` 裁判系统帧（含 game_time / sentry_posture）与 `0x16` 底盘数据帧（含一条"在动"的
   合成底盘速度，可选 `--chassis-echo` 回显速度指令），速率由 `--referee-hz` / `--chassis-hz`
   指定。帧头/ID/载荷/CRC16/帧尾/转义全部按仓库实现逐字节复刻。
2. **收（驱动 -> ECU）**：解析驱动发出的 `0x20` 底盘控制帧（ChassisControl），做完整反转义 +
   长度/ID/帧尾/CRC16 校验，以 1~2 Hz 打印 `vx,vy,wz,posture` + 帧尺寸 + 滑动帧率 +
   `crc_bad` / 丢弃 / 未知 ID 计数。
3. **对表（--sign-check）**：同进程用 rclpy 订阅 `/sentry/velocity_command` 与
   `/sentry/posture_command`，逐帧断言串口帧里的值与最后下发的指令**逐位相等**（同符号、同单位）。
   驱动侧是 `float` 直拷、`uint8` 直拷（`transfer.cpp` 的 `send_chassis_control()`），
   没有任何缩放或取整，所以实测 Δ=0.000000，`--tol` 只需覆盖浮点比较误差。
4. **故障注入**：`--corrupt-every`（载荷改哨兵值、帧结构不变 -> CRC 必须不符）、
   `--garbage-every`（发长度不足的畸形帧）、`--pause-after/--pause-for`（停发 ECU 侧帧，
   考驱动的读线程/看门狗）、`--stop-velocity-after` / `--stop-posture-after`
   （停发速度/姿态指令，考驱动的 `velocity_timeout` / `posture_timeout`）。
5. **断言 + 退出码**：所有断言打印 `ASSERT <name> PASS/FAIL <detail>`，任一条 FAIL 则退出码非 0；
   最后一行 `RESULT: PASS|FAIL …` 供 `tools/bench_uart.sh` 汇总成 PASS/FAIL 表。

帧格式（逐字节复刻仓库实现，右侧是出处）
----------------------------------------
    +------+-----+----------------+----------------+------+
    | 0x7d | id  | payload (N B)  | crc16 (2B, LE) | 0x7e |   未转义长度 = 2 + N + 3
    +------+-----+----------------+----------------+------+
    header 0x7d / tail 0x7e / header=2 / tail16=3      serial_trans_tool.hpp:7-11
    转义：对下标 [1, len-2] 内等于 0x7d/0x7e/0x7f 的字节 -> 0x7f, (b-0x7d)
                                                      serial_trans_tool.hpp:45-56
    CRC16：poly 0x1021（反射表实现）、初值 0xFFFF、无终异或，覆盖 id+payload
                              crc.hpp:21-47,49,55,68 / serial_trans_tool.hpp:28-32
    （接收侧的反转义在 uart_driver_node.cpp:181-184；校验在 serial_trans_tool.hpp:60-70,88-105）
    0x11 RefereeData    payload 25B -> 帧 30B        struct_def.hpp:7-24
    0x16 ChassisData    payload 24B -> 帧 29B        struct_def.hpp:27-39
    0x20 ChassisControl payload 17B -> 帧 22B        struct_def.hpp:43-54

    ChassisControl = float vx, float vy, float wz, u8 ph1, u8 sentry_posture, u8 ph2, u8 ph3, u8 ph4
    即 3*4 + 5 = 17 B（`ph` 占位全部置 0，见 struct_def.hpp:43-54 / transfer.cpp:113-116）。

用法
----
    # 只做纯协议自检（不需要 ROS / 串口，会打印推导出的字节与标准 CRC 校验值）
    python3 tools/fake_ecu.py --selftest

    # 台架（推荐）：一条命令起设备对 + 驱动 + 假 ECU
    bash tools/bench_uart.sh

    # 单独跑（需要驱动已连到同一 pty 对端，且本机 DDS 用 SHM）
    export FASTDDS_BUILTIN_TRANSPORTS=SHM
    python3 tools/fake_ecu.py --link /tmp/ttyFAKE_ECU --duration 10 --sign-check

说明
----
- 默认会像 planner/决策一样发布 `/sentry/velocity_command` 与 `/sentry/posture_command`
  （`--no-cmd` 可关掉），命令值在 3 组预设间轮换（`--cmd-hold` 秒一组），其中一组刻意让
  float32 小端字节里出现 0x7d/0x7e/0x7f，保证驱动的反转义路径每轮都被走到。
- 所有 `--*-after SEC` 都以 **T0 = 收到驱动第一帧有效控制帧的时刻** 为基准（T0 会打印出来），
  这样驱动/设备对启动慢一点也不会让场景时序跑偏。
"""

import argparse
import math
import os
import select
import signal
import struct
import sys
import time

# --------------------------------------------------------------------------- #
# 协议常量（出处见 docstring / 注释）
# --------------------------------------------------------------------------- #
SERIAL_START = 0x7D                                    # serial_trans_tool.hpp:7
SERIAL_END = 0x7E                                      # serial_trans_tool.hpp:8
ESCAPE_MARK = 0x7F                                     # serial_trans_tool.hpp:49
SERIAL_HEADER_SIZE = 2                                 # serial_trans_tool.hpp:9
SERIAL_TAIL_SIZE16 = 3                                 # serial_trans_tool.hpp:10

ID_REFEREE = 0x11                                      # struct_def.hpp:7
ID_CHASSIS_DATA = 0x16                                 # struct_def.hpp:27
ID_CHASSIS_CONTROL = 0x20                              # struct_def.hpp:43

PAYLOAD_SIZE = {ID_REFEREE: 25, ID_CHASSIS_DATA: 24, ID_CHASSIS_CONTROL: 17}
FRAME_SIZE = {k: SERIAL_HEADER_SIZE + n + SERIAL_TAIL_SIZE16 for k, n in PAYLOAD_SIZE.items()}

# packed struct 的等价 small-endian 打包格式（x86 / ARM 小端一致）
FMT_REFEREE = "<4HBH6B2f"          # ph1..ph3 u16, game_time u16, ph4 u8, ph5 u16, ph6..ph10 u8, ph11/12 f32
FMT_CHASSIS_DATA = "<5fh2B"        # vx,vy,wz,ph1,ph2 f32, ph3 i16, ph4,ph5 u8
FMT_CHASSIS_CONTROL = "<3f5B"      # vx,vy,wz f32, ph1, posture, ph2, ph3, ph4 u8

ESC_BYTES = (SERIAL_START, SERIAL_END, ESCAPE_MARK)


def bit_reverse16(x):
    r = 0
    for i in range(16):
        r |= ((x >> i) & 1) << (15 - i)
    return r


def _build_crc16_table(poly=0x1021):
    """与 crc.hpp 的 CRC16 构造函数逐行等价的表。"""
    tpoly = bit_reverse16(poly)
    table = []
    for i in range(256):
        crc, c = 0, i
        for _ in range(8):
            if (crc ^ c) & 0x0001:
                crc = (crc >> 1) ^ tpoly
            else:
                crc >>= 1
            c >>= 1
        table.append(crc & 0xFFFF)
    return tuple(table)


_CRC16_TABLE = _build_crc16_table(0x1021)


def crc16_ccitt(data, init=0xFFFF):
    """crc.hpp:49/55 的 `CRC16::check_sum()`（反射表 + 初值 0xFFFF，见 crc.hpp:21-47,68）。"""
    crc = init
    for b in data:
        crc = (crc >> 8) ^ _CRC16_TABLE[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


def escape_frame(raw):
    """对 raw[1:-1] 内的 0x7d/0x7e/0x7f 做转义，返回 (转义后字节, 转义字节数)。"""
    out = bytearray()
    out.append(raw[0])
    n_esc = 0
    for i in range(1, len(raw) - 1):
        b = raw[i]
        if b in ESC_BYTES:
            out.append(ESCAPE_MARK)
            out.append(b - SERIAL_START)
            n_esc += 1
        else:
            out.append(b)
    out.append(raw[-1])
    return bytes(out), n_esc


def build_frame(msg_id, payload, crc_override=None):
    """按仓库实现组一帧。crc_override 用于注入坏帧（载荷改了、CRC 仍按原载荷算）。

    返回 (线上字节, 未转义原始帧, 转义字节数)。
    """
    body = bytes([msg_id]) + bytes(payload)
    crc = crc16_ccitt(body) if crc_override is None else crc_override
    raw = bytes([SERIAL_START]) + body + struct.pack("<H", crc) + bytes([SERIAL_END])
    escaped, n_esc = escape_frame(raw)
    return escaped, raw, n_esc


def count_esc_bytes(raw):
    return sum(1 for b in raw[1:-1] if b in ESC_BYTES)


class FrameParser:
    """串口字节流 -> 未转义整帧（与 uart_driver_node.cpp:read_from_port 的状态机等价）。"""

    def __init__(self, max_len=256):
        self.max_len = max_len
        self.reset()
        self.discarded = 0
        self.resyncs = 0
        self.oversize = 0

    def reset(self):
        self.buf = bytearray()
        self.in_frame = False
        self.esc = False

    def feed(self, chunk):
        frames = []
        for b in chunk:
            if not self.in_frame:
                if b == SERIAL_START:
                    self.buf = bytearray([b])
                    self.in_frame = True
                    self.esc = False
                else:
                    self.discarded += 1
                continue
            if self.esc:
                self.buf.append((b + SERIAL_START) & 0xFF)   # uart_driver_node.cpp:181-184 的还原
                self.esc = False
                continue
            if b == ESCAPE_MARK:
                self.esc = True
                continue
            if b == SERIAL_END:
                self.buf.append(b)
                self.in_frame = False
                frames.append(bytes(self.buf))
                self.buf = bytearray()
                continue
            if b == SERIAL_START:
                # 合法帧体内不可能出现原始 0x7d（一定被转义），说明前面丢了字节 -> 重新同步
                self.resyncs += 1
                self.discarded += max(0, len(self.buf) - 1)
                self.buf = bytearray([b])
                continue
            self.buf.append(b)
            if len(self.buf) > self.max_len:
                self.oversize += 1
                self.discarded += len(self.buf)
                self.reset()
        return frames


def validate_chassis_control(frame):
    """校验并解出 ChassisControl（镜像 parse_serial_struct + buffer_check_valid，crc16 模式）。

    返回 (ok, reason, (vx, vy, wz, posture) 或 None, crc_fallback_ok)
    """
    if len(frame) != FRAME_SIZE[ID_CHASSIS_CONTROL]:
        return False, "size", None, False
    if frame[0] != SERIAL_START or frame[1] != ID_CHASSIS_CONTROL or frame[-1] != SERIAL_END:
        return False, "head/id/tail", None, False
    crc_val = struct.unpack_from("<H", frame, len(frame) - SERIAL_TAIL_SIZE16)[0]
    crc_chk = crc16_ccitt(frame[1:len(frame) - SERIAL_TAIL_SIZE16])
    fallback = False
    if crc_chk != crc_val:
        # 镜像 buffer_check_valid：再试一次「只算载荷」的 CRC
        crc_chk = crc16_ccitt(frame[SERIAL_HEADER_SIZE:len(frame) - SERIAL_TAIL_SIZE16])
        fallback = True
        if crc_chk != crc_val:
            return False, "crc", None, False
    vx, vy, wz, _ph1, posture, _ph2, _ph3, _ph4 = struct.unpack_from(FMT_CHASSIS_CONTROL, frame, 2)
    return True, "ok", (vx, vy, wz, posture), fallback


def f32_le_bytes(v):
    return struct.pack("<f", float(v))


def f32_from_le_bytes(b):
    return struct.unpack("<f", bytes(b))[0]


# 预设指令：第三组的小端字节里刻意包含 0x7d/0x7e/0x7f，强制走驱动的转义/反转义路径
ESCAPE_PRESET = (
    f32_from_le_bytes([0x00, 0x00, 0x7D, 0x43]),   # float32 0x437d0000 = 253.0
    f32_from_le_bytes([0x00, 0x00, 0x7E, 0x43]),   # float32 0x437e0000 = 254.0
    f32_from_le_bytes([0x00, 0x00, 0x7F, 0x43]),   # float32 0x437f0000 = 255.0
)
CMD_PRESETS = (
    (300.0, -120.0, 45.0),
    (-250.5, 180.0, -90.0),
    ESCAPE_PRESET,
)


class Sample:
    __slots__ = ("t", "vx", "vy", "wz", "posture", "raw_size", "esc", "reason")

    def __init__(self, t, vx, vy, wz, posture, raw_size, esc, reason):
        self.t = t
        self.vx = vx
        self.vy = vy
        self.wz = wz
        self.posture = posture
        self.raw_size = raw_size
        self.esc = esc
        self.reason = reason

    @property
    def zeros(self):
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


class CmdSample:
    __slots__ = ("t", "vx", "vy", "wz", "posture")

    def __init__(self, t, vx, vy, wz, posture):
        self.t = t
        self.vx = vx
        self.vy = vy
        self.wz = wz
        self.posture = posture


class FakeEcu:
    def __init__(self, args):
        self.args = args
        self.start_t = time.monotonic()
        self.t0 = None                       # 第一帧驱动控制帧到达时刻
        self.stop = False
        self.interrupted = False
        self.ser = None
        self.link_real = None
        self.parser = FrameParser()

        # RX（驱动 -> ECU）
        self.frames = []                     # 全部已识别的 0x20 帧（含坏帧）
        self.good = []                       # 校验通过的帧
        self.rx_bad_size = 0
        self.rx_bad_crc = 0
        self.rx_unk_id = 0
        self.rx_crc_fallback = 0
        self.rx_esc_frames = 0
        self.rx_esc_bytes = 0
        self.io_errors = 0

        # TX（ECU -> 驱动）
        self.tx_count = {ID_REFEREE: 0, ID_CHASSIS_DATA: 0}
        self.tx_times = {ID_REFEREE: [], ID_CHASSIS_DATA: []}   # 滑动窗口帧率用
        self.corrupt_sent = 0
        self.garbage_sent = 0
        self.tx_last = dict.fromkeys((ID_REFEREE, ID_CHASSIS_DATA))

        # 场景
        self.pause_begin = None
        self.pause_end = None
        self.in_pause = False
        self.pause_skipped_ticks = 0     # 暂停期间被跳过的发送节拍
        self.tx_during_pause = 0         # 暂停期间实际写出的帧（必须为 0）
        self.velocity_stop_t = None
        self.posture_stop_t = None
        self.events = []                      # (t, name, detail)

        # 命令侧
        self.node = None
        self.cmds = []
        self.cmd_count = 0
        self.cmd_presets_used = []
        self.cmds_with_escape = 0

        # 对表 / 底盘回读
        self.sign_checks = 0
        self.sign_started = False        # 驱动吃到第一条指令后才开始对表
        self.sign_mismatch = 0
        self.sign_latest_ok = 0
        self.sign_max_dev = 0.0
        self.sign_examples = []
        self.chassis_msgs = []
        self.poison_seen = 0
        self.chassis_min = None
        self.chassis_max = None

    # ------------------------------------------------------------------ #
    def log(self, fmt, *a):
        print(f"[ecu {time.monotonic() - self.start_t:6.2f}s] " + (fmt % a if a else fmt), flush=True)

    def event(self, name, detail=""):
        t = time.monotonic() - self.start_t
        self.events.append((t, name, detail))
        ref = f"T0+{(time.monotonic() - self.t0):.2f}s" if self.t0 else "T0=?"
        self.log("EVENT %s t=%s %s", name, ref, detail)

    # ------------------------------------------------------------------ #
    def open_link(self):
        try:
            import serial
        except ImportError as exc:
            print(f"pyserial 不可用: {exc}", file=sys.stderr)
            return False
        deadline = time.monotonic() + self.args.link_timeout
        last = None
        while time.monotonic() < deadline:
            if os.path.exists(self.args.link):
                try:
                    self.ser = serial.Serial(self.args.link, self.args.baud,
                                             timeout=0, write_timeout=0.5)
                    self.link_real = os.path.realpath(self.args.link)
                    self.log("已打开链路 %s -> %s @%d baud", self.args.link, self.link_real, self.args.baud)
                    return True
                except Exception as exc:      # 设备还没就绪：继续等
                    last = exc
            time.sleep(0.1)
        print(f"链路 {self.args.link} 在 {self.args.link_timeout:.1f}s 内打不开: {last}", file=sys.stderr)
        return False

    # ------------------------------------------------------------------ #
    def setup_ros(self):
        needs_ros = (self.args.sign_check or self.args.check_chassis
                     or not self.args.no_cmd)
        if not needs_ros:
            return True
        try:
            import rclpy
            from rm_msgs.msg import ChassisData, PostureCommand, VelocityCommand
            from std_msgs.msg import Header
        except ImportError as exc:
            print(f"ROS 环境不可用: {exc}\n请先 source /opt/ros/kilted/setup.bash && "
                  f"source install/setup.bash", file=sys.stderr)
            return False
        self.rclpy = rclpy
        self.VelocityCommand, self.PostureCommand = VelocityCommand, PostureCommand
        self.Header = Header
        rclpy.init()
        self.node = rclpy.create_node("fake_ecu")
        self.vel_pub = self.post_pub = None
        if not self.args.no_cmd:
            qos = 10
            self.vel_pub = self.node.create_publisher(VelocityCommand, self.args.velocity_topic, qos)
            self.post_pub = self.node.create_publisher(PostureCommand, self.args.posture_topic, qos)
        if self.args.sign_check:
            self.node.create_subscription(VelocityCommand, self.args.velocity_topic,
                                          self.on_velocity, 10)
            self.node.create_subscription(PostureCommand, self.args.posture_topic,
                                          self.on_posture, 10)
        if self.args.check_chassis:
            self.node.create_subscription(ChassisData, self.args.chassis_topic,
                                          self.on_chassis, 10)
        self.log("ROS: domain_id=%s transport=%s node=fake_ecu 订阅=[%s%s]", os.environ.get("ROS_DOMAIN_ID", "?"),
                 os.environ.get("FASTDDS_BUILTIN_TRANSPORTS", "?"), self.args.velocity_topic,
                 f", {self.args.chassis_topic}]" if self.args.check_chassis else "]")
        return True

    # ---- ROS 回调（只用于对表/回读，不参与断言之外的逻辑） ---- #
    def on_velocity(self, _msg):
        pass      # 订阅只为让驱动的话题进入发现；断言做在串口帧路径上，不依赖回调时机

    def on_posture(self, _msg):
        pass

    def on_chassis(self, msg):
        t = time.monotonic() - self.start_t
        self.chassis_msgs.append((t, float(msg.vx), float(msg.vy), float(msg.wz)))
        for v in (float(msg.vx), float(msg.vy), float(msg.wz)):
            if abs(v - self.args.poison_vx) < 1e-3:
                self.poison_seen += 1
        self.chassis_min = float(msg.vx) if self.chassis_min is None else min(self.chassis_min, float(msg.vx))
        self.chassis_max = float(msg.vx) if self.chassis_max is None else max(self.chassis_max, float(msg.vx))

    # ------------------------------------------------------------------ #
    def current_preset(self):
        if self.args.vx is not None or self.args.vy is not None or self.args.wz is not None:
            return (self.args.vx or 0.0, self.args.vy or 0.0, self.args.wz or 0.0), 0
        if not self.args.sign_check:
            return CMD_PRESETS[0], 0
        idx = int(self.cmd_phase()) % len(CMD_PRESETS)
        return CMD_PRESETS[idx], idx

    def cmd_phase(self):
        if self.t0 is None:
            return 0
        return (time.monotonic() - self.t0) / max(1e-3, self.args.cmd_hold)

    def publish_cmd(self):
        now = time.monotonic()
        vx, vy, wz = self.current_preset()[0]
        posture = self.args.posture
        if self.args.stop_velocity_after is not None and \
                self.t0 is not None and now - self.t0 >= self.args.stop_velocity_after:
            if self.velocity_stop_t is None:
                self.velocity_stop_t = now
                self.event("velocity_cmd_stopped", f"after={self.args.stop_velocity_after:.2f}s")
        if self.args.stop_posture_after is not None and \
                self.t0 is not None and now - self.t0 >= self.args.stop_posture_after:
            if self.posture_stop_t is None:
                self.posture_stop_t = now
                self.event("posture_cmd_stopped", f"after={self.args.stop_posture_after:.2f}s")

        if self.velocity_stop_t is None and self.vel_pub is not None:
            m = self.VelocityCommand()
            m.header = self.Header()
            m.vx, m.vy, m.wz = float(vx), float(vy), float(wz)
            self.vel_pub.publish(m)
        if self.posture_stop_t is None and self.post_pub is not None:
            p = self.PostureCommand()
            p.header = self.Header()
            p.posture = int(posture)
            self.post_pub.publish(p)

        if self.velocity_stop_t is not None and self.posture_stop_t is not None:
            return
        raw_frame = build_frame(ID_CHASSIS_CONTROL,
                                struct.pack(FMT_CHASSIS_CONTROL, vx, vy, wz, 0, posture, 0, 0, 0),
                                )[1]
        esc = count_esc_bytes(raw_frame)
        if esc:
            self.cmds_with_escape += 1
        self.cmds.append(CmdSample(now, float(vx), float(vy), float(wz), int(posture)))
        self.cmd_count += 1
        if not self.cmd_presets_used or self.cmd_presets_used[-1] != (vx, vy, wz):
            self.cmd_presets_used.append((vx, vy, wz))

    # ------------------------------------------------------------------ #
    def paused(self, now):
        a = self.args.pause_after
        if a is None or self.t0 is None:
            return False
        rel = now - self.t0
        if rel < a:
            return False
        if rel > a + self.args.pause_for:
            if self.pause_end is None:
                self.pause_end = now
                self.event("pause_end", f"skipped_ticks={self.pause_skipped_ticks}")
            return False
        if self.pause_begin is None:
            self.pause_begin = now
            self.event("pause_begin", f"duration={self.args.pause_for:.2f}s")
        return True

    def emit(self, msg_id, payload):
        if self.in_pause:
            self.tx_during_pause += 1
        escaped, _raw, _n_esc = build_frame(msg_id, payload)
        try:
            self.ser.write(escaped)
        except Exception as exc:
            self.io_errors += 1
            if self.io_errors <= 3:
                self.log("写串口失败: %s", exc)
            return
        self.record_tx(msg_id)

    def emit_referee(self):
        t = time.monotonic() - self.start_t
        game_time = int(t) & 0xFFFF
        # ph11 用小端字节含 0x7d7e7f 的浮点占位值，保证每帧都走一次转义路径
        ph11 = f32_from_le_bytes([0x7D, 0x7E, 0x7F, 0x00])
        payload = struct.pack(FMT_REFEREE, 0x1111, 0x2222, 0x3333, game_time, 0x44, 0x5555,
                              0x66, 0x67, 0x68, self.args.posture | 0x08, 0x69, 0x6A, ph11, 0.0)
        self.emit(ID_REFEREE, payload)

    def chassis_value(self):
        t = time.monotonic() - self.start_t
        if self.args.chassis_echo and self.cmds:
            c = self.cmds[-1]
            return (c.vx * self.args.chassis_echo_scale, c.vy * self.args.chassis_echo_scale,
                    c.wz * self.args.chassis_echo_scale)
        w = 2.0 * math.pi / max(0.1, self.args.chassis_cycle)
        a = self.args.chassis_speed
        return (a * math.cos(w * t), 0.5 * a * math.sin(w * t + 0.7), 30.0 * math.sin(w * t))

    def emit_chassis(self):
        idx = self.tx_count[ID_CHASSIS_DATA] + 1
        payload = struct.pack(FMT_CHASSIS_DATA, *self.chassis_value(), 0.0, 0.0, 0, 0, 0)

        if self.args.garbage_every and idx % self.args.garbage_every == 0:
            # 畸形帧：帧头/ID/帧尾齐，但长度远不足 -> 驱动必须丢弃并计数
            raw = bytes([SERIAL_START, ID_CHASSIS_DATA, 0x01, 0x02, 0x03, SERIAL_END])
            try:
                self.ser.write(raw)
            except Exception:
                self.io_errors += 1
                return
            self.record_tx(ID_CHASSIS_DATA)
            self.garbage_sent += 1
            if self.garbage_sent == 1:
                self.event("garbage_sent", f"first at chassis#{idx} bytes={raw.hex(' ')}")
            return

        if self.args.corrupt_every and idx % self.args.corrupt_every == 0:
            # 坏帧：载荷换成哨兵值、帧长/帧尾不变，CRC 仍按原载荷算 -> 必然校验失败、不得变成数据
            p = self.args.poison_vx
            poison = struct.pack(FMT_CHASSIS_DATA, p, -p, p, 0.0, 0.0, 0, 0, 0)
            crc_orig = crc16_ccitt(bytes([ID_CHASSIS_DATA]) + payload)
            raw = (bytes([SERIAL_START, ID_CHASSIS_DATA]) + poison
                   + struct.pack("<H", crc_orig) + bytes([SERIAL_END]))
            escaped, _ = escape_frame(raw)
            try:
                self.ser.write(escaped)
            except Exception:
                self.io_errors += 1
                return
            self.record_tx(ID_CHASSIS_DATA)
            self.corrupt_sent += 1
            if self.corrupt_sent == 1:
                self.event("corrupt_sent", f"first at chassis#{idx} poison_vx={p}")
            return

        self.emit(ID_CHASSIS_DATA, payload)

    # ------------------------------------------------------------------ #
    def on_serial(self):
        try:
            n = self.ser.in_waiting
            data = self.ser.read(n if n else 1)
        except Exception as exc:
            self.io_errors += 1
            if self.io_errors <= 3:
                self.log("读串口失败: %s", exc)
            return
        if not data:
            return
        for frame in self.parser.feed(data):
            self.handle_frame(frame)

    def handle_frame(self, frame):
        now = time.monotonic()
        msg_id = frame[1] if len(frame) > 1 else None
        if msg_id != ID_CHASSIS_CONTROL:
            self.rx_unk_id += 1
            self.frames.append(Sample(now, 0, 0, 0, 0, len(frame), 0, "unk_id"))
            return
        esc = count_esc_bytes(frame)
        ok, reason, vals, fallback = validate_chassis_control(frame)
        if ok:
            if fallback:
                self.rx_crc_fallback += 1
            if self.t0 is None:
                self.t0 = now
                self.log("T0 锚点：收到第一帧驱动控制帧（%s）", frame.hex(" "))
            vx, vy, wz, posture = vals
            s = Sample(now, vx, vy, wz, posture, len(frame), esc, "ok")
            self.good.append(s)
            self.frames.append(s)
            if esc:
                self.rx_esc_frames += 1
                self.rx_esc_bytes += esc
            self.sign_check_frame(s)
            self.watchdog_scan(s)
        else:
            if reason == "crc":
                self.rx_bad_crc += 1
            else:
                self.rx_bad_size += 1
            self.frames.append(Sample(now, 0, 0, 0, 0, len(frame), esc, reason))
            if self.rx_bad_crc + self.rx_bad_size <= 3:
                self.log("丢弃非法帧 reason=%s size=%dB raw=%s", reason, len(frame), frame.hex(" "))

    def sign_check_frame(self, s):
        """断言：串口帧的值必须等于某个「当时已经下发过」的指令样本（同符号同单位）。"""
        if not self.args.sign_check or not self.cmds:
            return
        if self.velocity_stop_t is not None and s.t - self.velocity_stop_t > self.args.tol_window:
            return                                  # 已停发指令：进入看门狗场景，不再对表
        if self.posture_stop_t is not None and s.t - self.posture_stop_t > self.args.tol_window:
            return
        best = None
        latest = None
        for c in reversed(self.cmds):
            if c.t > s.t + 1e-3:
                continue
            latest = latest or c
            d = max(abs(c.vx - s.vx), abs(c.vy - s.vy), abs(c.wz - s.wz),
                    abs(c.posture - s.posture))
            best = d if best is None else min(best, d)
        if best is None:
            return
        if not self.sign_started:
            # 启动宽限：DDS 发现要时间，驱动在收到第一条速度指令前只能发零速；
            # 以「第一帧匹配上」为起点，超过 --cmd-grace 还匹配不上就当成真失败（驱动没吃到指令）。
            anchor = self.t0 if self.t0 else self.cmds[0].t
            if best <= self.args.tol:
                self.sign_started = True
            elif time.monotonic() - anchor < self.args.cmd_grace:
                return
            else:
                self.sign_started = True
        self.sign_checks += 1
        self.sign_max_dev = max(self.sign_max_dev, best)
        if best > self.args.tol:
            self.sign_mismatch += 1
            if self.sign_mismatch <= 3:
                self.log("对表不符: 帧 vx=%+.3f vy=%+.3f wz=%+.3f posture=%d 最大偏差=%.6f",
                         s.vx, s.vy, s.wz, s.posture, best)
        else:
            if latest is not None and max(abs(latest.vx - s.vx), abs(latest.vy - s.vy),
                                          abs(latest.wz - s.wz),
                                          abs(latest.posture - s.posture)) <= self.args.tol:
                self.sign_latest_ok += 1
            if len(self.sign_examples) < 3:
                self.sign_examples.append(
                    f"cmd(vx={latest.vx:+.3f} vy={latest.vy:+.3f} wz={latest.wz:+.3f} "
                    f"posture={latest.posture}) -> frame(vx={s.vx:+.3f} vy={s.vy:+.3f} "
                    f"wz={s.wz:+.3f} posture={s.posture}) Δ={best:.6f}")

    def watchdog_scan(self, s):
        """记录「指令停发 -> 驱动看门狗生效」的首帧证据。"""
        if self.velocity_stop_t is not None and s.zeros:
            if not any(e[1] == "velocity_zero_first" for e in self.events):
                self.event("velocity_zero_first",
                           f"dt={s.t - self.velocity_stop_t:.3f}s (velocity_timeout={self.args.velocity_timeout:.2f}s) "
                           f"vx={s.vx:.3f} vy={s.vy:.3f} wz={s.wz:.3f}")
        if self.posture_stop_t is not None and s.posture == self.args.expect_posture and \
                not any(e[1] == "posture_move_first" for e in self.events):
            self.event("posture_move_first",
                       f"dt={s.t - self.posture_stop_t:.3f}s (posture_timeout={self.args.posture_timeout:.2f}s) "
                       f"posture={s.posture}")

    # ------------------------------------------------------------------ #
    def periodic_log(self):
        now = time.monotonic()
        lo = now - self.args.rate_window
        win = [s for s in self.good if s.t >= lo]
        rate = len(win) / max(1e-6, min(self.args.rate_window, now - self.start_t))
        last = self.good[-1] if self.good else None
        tail = ""
        if last is not None:
            tail = (f"last vx={last.vx:+9.3f} vy={last.vy:+9.3f} wz={last.wz:+9.3f} "
                    f"posture={last.posture} size={last.raw_size}B esc={last.esc}")
        else:
            tail = "还没收到任何 0x20 帧"
        self.log("rx=%d (%.1fHz) crc_bad=%d size_bad=%d unk_id=%d resync=%d | %s | "
                 "tx ref=%d(%.1fHz) chas=%d(%.1fHz) corrupt=%d garbage=%d",
                 len(self.frames), rate, self.rx_bad_crc, self.rx_bad_size, self.rx_unk_id,
                 self.parser.resyncs, tail, self.tx_count[ID_REFEREE],
                 self.rate_of(ID_REFEREE, lo), self.tx_count[ID_CHASSIS_DATA],
                 self.rate_of(ID_CHASSIS_DATA, lo), self.corrupt_sent, self.garbage_sent)

    def record_tx(self, msg_id):
        now = time.monotonic()
        self.tx_count[msg_id] += 1
        times = self.tx_times[msg_id]
        times.append(now)
        cut = now - self.args.rate_window
        while times and times[0] < cut:
            times.pop(0)

    def rate_of(self, msg_id, lo):
        """本窗口内的实际发送帧率（滑动窗口，只用于日志观察）。"""
        n = sum(1 for t in self.tx_times[msg_id] if t >= lo)
        return n / max(1e-6, min(self.args.rate_window, time.monotonic() - self.start_t))

    # ------------------------------------------------------------------ #
    def run(self):
        if not self.open_link():
            return 1
        if not self.setup_ros():
            return 2
        self.install_signals()
        self.log("参数: referee_hz=%.1f chassis_hz=%.1f cmd_hz=%.1f duration=%.1f link=%s",
                 self.args.referee_hz, self.args.chassis_hz, self.args.cmd_hz, self.args.duration,
                 self.link_real)
        self.log("推导帧格式: 0x11 ->%dB  0x16 ->%dB  0x20 ->%dB （见 tools/fake_ecu.py 头部）",
                 FRAME_SIZE[ID_REFEREE], FRAME_SIZE[ID_CHASSIS_DATA], FRAME_SIZE[ID_CHASSIS_CONTROL])

        next_ref = self.start_t
        next_chas = self.start_t
        next_cmd = self.start_t
        next_log = self.start_t + 1.0
        t_end = self.start_t + self.args.duration
        first_deadline = self.start_t + self.args.first_frame_timeout

        while not self.stop and time.monotonic() < t_end:
            now = time.monotonic()
            self.on_serial()
            if self.node is not None:
                self.rclpy.spin_once(self.node, timeout_sec=0.0)

            if self.paused(now):
                self.in_pause = True
                if now >= next_ref:
                    next_ref = now + 1.0 / self.args.referee_hz
                    self.pause_skipped_ticks += 1
                if now >= next_chas:
                    next_chas = now + 1.0 / self.args.chassis_hz
                    self.pause_skipped_ticks += 1
            else:
                self.in_pause = False
                if now >= next_ref:
                    self.emit_referee()
                    next_ref += 1.0 / self.args.referee_hz
                    if next_ref < now - 0.2:
                        next_ref = now + 1.0 / self.args.referee_hz
                if now >= next_chas:
                    self.emit_chassis()
                    next_chas += 1.0 / self.args.chassis_hz
                    if next_chas < now - 0.2:
                        next_chas = now + 1.0 / self.args.chassis_hz

            if now >= next_cmd and self.args.cmd_hz > 0:
                self.publish_cmd()
                next_cmd += 1.0 / self.args.cmd_hz

            if now >= next_log:
                next_log += 1.0 / max(1e-3, self.args.log_hz) if self.args.log_hz > 0 else 1e9
                if self.args.log_hz > 0:
                    self.periodic_log()

            if self.t0 is None and now > first_deadline:
                self.log("警告：%.1f s 内没收到驱动的任何 0x20 帧（驱动起来了吗？serial_name 对吗？）",
                         self.args.first_frame_timeout)
                break
            time.sleep(0.002)

        return self.finish()

    def install_signals(self):
        def handler(signum, _frame):
            self.interrupted = True
            self.stop = True
            print(f"\n[ecu] 收到信号 {signum}，收尾并打印报告", flush=True)

        for s in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(s, handler)
            except ValueError:
                pass

    # ------------------------------------------------------------------ #
    def window_rate(self, lo, hi):
        lo = max(lo, self.start_t)
        if hi <= lo:
            return 0.0
        n = sum(1 for s in self.good if lo <= s.t <= hi)
        return n / (hi - lo)

    def finish(self):
        end = time.monotonic()
        checks = []
        H = self.args.min_rate_hz
        warm_lo = (self.t0 + self.args.rate_warmup) if self.t0 else (self.start_t + self.args.rate_warmup)
        rate = self.window_rate(warm_lo, end)
        overall = 0.0
        if len(self.good) >= 2:
            overall = (len(self.good) - 1) / max(1e-6, self.good[-1].t - self.good[0].t)

        def add(name, ok, detail):
            checks.append((name, bool(ok), detail))

        add("link_alive", self.t0 is not None,
            f"first_frame={'T0+' + format(self.t0 - self.start_t, '.2f') + 's' if self.t0 else 'NONE'}")
        add("link_rate", rate >= H and self.args.min_rate_hz > 0,
            f"{rate:.1f}Hz >= {H:.1f}Hz (窗口 {max(0.0, end - warm_lo):.1f}s, 全程 {overall:.1f}Hz)")
        proto_ok = (self.rx_bad_crc == 0 and self.rx_bad_size == 0 and self.rx_unk_id == 0
                    and self.parser.resyncs == 0 and self.parser.oversize == 0)
        add("rx_protocol", proto_ok,
            f"帧尺寸=22B crc_bad={self.rx_bad_crc} size_bad={self.rx_bad_size} "
            f"unk_id={self.rx_unk_id} resync={self.parser.resyncs} oversize={self.parser.oversize} "
            f"转义帧={self.rx_esc_frames}(共{self.rx_esc_bytes}B)")
        if self.cmds_with_escape:
            add("escape_path", self.rx_esc_bytes > 0,
                f"指令侧有 {self.cmds_with_escape} 帧含待转义字节 -> 实测收到 {self.rx_esc_bytes}B 转义数据")
        if self.args.sign_check:
            add("sign_check", self.sign_mismatch == 0 and self.sign_checks >= 20,
                f"checked={self.sign_checks} mismatch={self.sign_mismatch} "
                f"max|Δ|={self.sign_max_dev:.6f} tol={self.args.tol:g} "
                f"匹配最新指令={self.sign_latest_ok}/{self.sign_checks}")
        if self.args.check_chassis:
            end_rel = end - self.start_t
            lo = end_rel - self.args.chassis_window
            cr = sum(1 for m in self.chassis_msgs if m[0] >= lo) / max(1e-6, min(self.args.chassis_window, end_rel))
            add("chassis_rate", cr >= self.args.min_chassis_hz,
                f"/chassis_data {cr:.1f}Hz >= {self.args.min_chassis_hz:.1f}Hz (最后 {self.args.chassis_window:.1f}s, "
                f"共 {len(self.chassis_msgs)} 条)")
            if self.corrupt_sent or self.garbage_sent:
                add("no_fake_data", self.poison_seen == 0,
                    f"坏帧 {self.corrupt_sent} 条 / 畸形帧 {self.garbage_sent} 条已注入, "
                    f"哨兵 vx={self.args.poison_vx:g} 出现在 /chassis_data 的次数={self.poison_seen}")
        if self.args.pause_after is not None:
            if self.pause_begin is None:
                add("pause_injected", False, "暂停窗口没到（--pause-after 太大？）")
            else:
                after = [s for s in self.good if s.t >= (self.pause_end or self.pause_begin) + 0.5]
                add("pause_injected", self.tx_during_pause == 0 and bool(self.pause_end),
                    f"暂停期间 ECU 侧实际发出帧数={self.tx_during_pause} (应为 0), 跳过节拍={self.pause_skipped_ticks}")
                add("pause_resume", len(after) >= 5,
                    f"恢复后收到驱动控制帧={len(after)} 条，暂停期间收到={len([s for s in self.good if self.pause_begin <= s.t <= (self.pause_end or self.pause_begin)])} 条")
        if self.args.stop_velocity_after is not None:
            add("velocity_watchdog", *self.check_velocity_watchdog())
        if self.args.stop_posture_after is not None:
            add("posture_watchdog", *self.check_posture_watchdog())

        self.print_report(checks, rate, overall)
        failed = [c for c in checks if not c[1]]
        if self.interrupted:
            return 130
        return 1 if failed else 0

    def check_velocity_watchdog(self):
        stop = self.velocity_stop_t
        if stop is None:
            return False, "指令停发点没到（--stop-velocity-after 太大？）"
        pre = [s for s in self.good if s.t < stop and not s.zeros]
        first_zero = next((s for s in self.good if s.t >= stop and s.zeros), None)
        deadline = stop + self.args.velocity_timeout + self.args.watchdog_margin
        if first_zero is None:
            return False, f"停发 {self.args.velocity_timeout:.2f}s+ 后仍未见零速帧（--duration 太短？）"
        late = [s for s in self.good if s.t >= deadline and not s.zeros]
        detail = (f"停发前非零帧={len(pre)} 首帧零速 dt={first_zero.t - stop:.3f}s "
                  f"(velocity_timeout={self.args.velocity_timeout:.2f}s, 上限{self.args.velocity_timeout + self.args.watchdog_margin:.2f}s), "
                  f"超期后仍非零帧={len(late)}")
        ok = bool(pre) and first_zero.t <= deadline and not late
        return ok, detail

    def check_posture_watchdog(self):
        stop = self.posture_stop_t
        if stop is None:
            return False, "姿态指令停发点没到（--stop-posture-after 太大？）"
        pre = [s for s in self.good if s.t < stop]
        deadline = stop + self.args.posture_timeout + self.args.watchdog_margin
        hit = next((s for s in self.good if s.t >= stop and s.posture == self.args.expect_posture), None)
        if hit is None:
            return False, f"停发 {self.args.posture_timeout:.2f}s+ 后姿态未回到 {self.args.expect_posture}"
        detail = (f"停发前姿态={sorted({s.posture for s in pre})} 回到 {self.args.expect_posture} "
                  f"dt={hit.t - stop:.3f}s (posture_timeout={self.args.posture_timeout:.2f}s, "
                  f"上限{self.args.posture_timeout + self.args.watchdog_margin:.2f}s)")
        ok = hit.t <= deadline
        return ok, detail

    def print_report(self, checks, rate, overall):
        a = self.args
        lines = []
        lines.append("=" * 108)
        if self.interrupted:
            lines.append("!! 运行被信号中断（SIGINT/SIGTERM），下面是已完成的统计与断言")
        lines.append("fake ECU 报告 / FAKE-ECU SUMMARY    "
                     f"link={self.link_real}  duration={time.monotonic() - self.start_t:.2f}s  "
                     f"T0={('T0+' + format(self.t0 - self.start_t, '.2f') + 's') if self.t0 else 'NONE'}")
        lines.append(f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '?')}  "
                     f"FASTDDS_BUILTIN_TRANSPORTS={os.environ.get('FASTDDS_BUILTIN_TRANSPORTS', '?')}")
        lines.append("-" * 108)
        lines.append(f"RX 驱动->ECU 0x20: 识别 {len(self.frames)} 帧 / 有效 {len(self.good)} 帧  "
                     f"全程 {overall:.1f}Hz  {max(0.0, rate if rate else 0):.1f}Hz(稳态窗口)")
        lines.append(f"  帧尺寸 22B(未转义)  含转义帧 {self.rx_esc_frames} 条(共 {self.rx_esc_bytes}B)  "
                     f"crc_bad={self.rx_bad_crc} size_bad={self.rx_bad_size} unk_id={self.rx_unk_id} "
                     f"resync={self.parser.resyncs} crc_fallback_ok={self.rx_crc_fallback} "
                     f"io_err={self.io_errors}")
        lines.append(f"TX ECU->驱动 0x11: {self.tx_count[ID_REFEREE]} 帧(30B, {a.referee_hz:.1f}Hz)  "
                     f"0x16: {self.tx_count[ID_CHASSIS_DATA]} 帧(29B, {a.chassis_hz:.1f}Hz)  "
                     f"坏帧={self.corrupt_sent} 畸形帧={self.garbage_sent} "
                     f"暂停期实际发送={self.tx_during_pause}(跳过节拍={self.pause_skipped_ticks})")
        if a.sign_check:
            lines.append(f"对表 sign-check: checked={self.sign_checks} mismatch={self.sign_mismatch} "
                         f"max|Δ|={self.sign_max_dev:.6f} tol={a.tol:g}  匹配最新指令={self.sign_latest_ok}/{self.sign_checks}")
            for ex in self.sign_examples:
                lines.append("  例: " + ex)
        if a.check_chassis:
            lines.append(f"/chassis_data 回读: {len(self.chassis_msgs)} 条  vx∈[{self.chassis_min}, {self.chassis_max}]  "
                         f"哨兵命中={self.poison_seen}")
        if self.cmd_count:
            lines.append(f"下发指令 {self.cmd_count} 条 @{a.cmd_hz:.1f}Hz, 预设切换 {len(self.cmd_presets_used)} 组: "
                         + " | ".join(f"({v[0]:g},{v[1]:g},{v[2]:g})" for v in self.cmd_presets_used))
        if self.events:
            lines.append("事件:")
            for t, name, detail in self.events:
                lines.append(f"  EVENT {name} t={t:.2f}s {detail}")
        lines.append("-" * 108)
        for name, ok, detail in checks:
            lines.append(f"ASSERT {name:<18} {'PASS' if ok else 'FAIL'}  {detail}")
        n_ok = sum(1 for c in checks if c[1])
        failed = [c[0] for c in checks if not c[1]]
        verdict = "PASS" if not failed else "FAIL"
        lines.append("-" * 108)
        lines.append(f"RESULT: {verdict}  asserts={n_ok}/{len(checks)}  frames={len(self.good)}  "
                     f"rate={rate:.1f}Hz  size=22B  crc_bad={self.rx_bad_crc}  "
                     f"corrupt_sent={self.corrupt_sent}  garbage_sent={self.garbage_sent}  "
                     f"poison_seen={self.poison_seen}  sign_mismatch={self.sign_mismatch}"
                     + (f"  FAILED={failed}" if failed else ""))
        lines.append("=" * 108)
        print("\n".join(lines), flush=True)

    def close(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            try:
                if self.rclpy.ok():
                    self.rclpy.shutdown()
            except Exception:
                pass


# --------------------------------------------------------------------------- #
# 纯协议自检（不需要 ROS / 串口）
# --------------------------------------------------------------------------- #
def run_selftest():
    ok = True
    out = []

    def check(name, cond, detail):
        nonlocal ok
        ok = ok and bool(cond)
        out.append(f"ASSERT {name:<22} {'PASS' if cond else 'FAIL'}  {detail}")

    check("crc16_standard", crc16_ccitt(b"123456789") == 0x6F91,
          f"crc16_ccitt(b'123456789')=0x{crc16_ccitt(b'123456789'):04X} (CRC-16/MCRF4XX 标准校验值 0x6F91)")

    sizes = {hex(k): (struct.calcsize(f), FRAME_SIZE[k])
             for k, f in ((ID_REFEREE, FMT_REFEREE), (ID_CHASSIS_DATA, FMT_CHASSIS_DATA),
                          (ID_CHASSIS_CONTROL, FMT_CHASSIS_CONTROL))}
    check("struct_sizes", all(p == PAYLOAD_SIZE[int(k, 16)] and f == FRAME_SIZE[int(k, 16)]
                              for k, (p, f) in sizes.items()),
          f"packed 载荷/帧: " + ", ".join(f"{k}:{p}B->{f}B" for k, (p, f) in sizes.items()))

    ctrl = struct.pack(FMT_CHASSIS_CONTROL, 300.0, -120.0, 45.0, 0, 1, 0, 0, 0)
    escaped, raw, n_esc = build_frame(ID_CHASSIS_CONTROL, ctrl)
    check("frame_20_bytes", len(raw) == 22 and raw[0] == 0x7D and raw[1] == 0x20 and raw[-1] == 0x7E,
          f"raw={raw.hex(' ')}  (len={len(raw)})")
    crc_expect = crc16_ccitt(raw[1:19])
    check("frame_20_crc", struct.unpack_from("<H", raw, 19)[0] == crc_expect,
          f"crc16(id+payload)=0x{crc_expect:04X} -> 帧内 LE 字节 {raw[19:21].hex(' ')}")

    r_ok, r_why, vals, fb = validate_chassis_control(raw)
    check("roundtrip_parse", r_ok and vals == (300.0, -120.0, 45.0, 1) and not fb,
          f"解析回 (vx,vy,wz,posture)={vals} reason={r_why} crc_fallback={fb}")

    esc_ctrl = struct.pack(FMT_CHASSIS_CONTROL, *ESCAPE_PRESET, 0, 1, 0, 0, 0)
    esc_escaped, esc_raw, esc_n = build_frame(ID_CHASSIS_CONTROL, esc_ctrl)
    parser = FrameParser()
    got = parser.feed(esc_escaped)
    check("escape_roundtrip", len(got) == 1 and got[0] == esc_raw and esc_n == 3,
          f"载荷字节={esc_ctrl.hex(' ')} 转义 {esc_n} 处 -> 线上长度 {len(esc_raw)}B->{len(esc_escaped)}B, "
          f"解析回原始 {len(got[0])}B")
    ok2, why2, vals2, _ = validate_chassis_control(got[0])
    check("escape_values", ok2 and vals2[:3] == ESCAPE_PRESET,
          f"{vals2[:3]} == 预设 {ESCAPE_PRESET}")

    # 坏帧：载荷改哨兵、CRC 按原载荷 -> 必须被丢弃
    poison = struct.pack(FMT_CHASSIS_DATA, 123456.0, -123456.0, 123456.0, 0.0, 0.0, 0, 0, 0)
    good_chassis = struct.pack(FMT_CHASSIS_DATA, 250.0, 0.0, 10.0, 0.0, 0.0, 0, 0, 0)
    crc_orig = crc16_ccitt(bytes([ID_CHASSIS_DATA]) + good_chassis)
    bad_raw = (bytes([SERIAL_START, ID_CHASSIS_DATA]) + poison + struct.pack("<H", crc_orig)
               + bytes([SERIAL_END]))
    crc_chassis = crc16_ccitt(bytes([ID_CHASSIS_DATA]) + poison)
    check("corrupt_detected", crc_chassis != crc_orig and len(bad_raw) == FRAME_SIZE[ID_CHASSIS_DATA],
          f"哨兵载荷 CRC=0x{crc_chassis:04X} != 帧内 0x{crc_orig:04X}（帧长仍 {len(bad_raw)}B -> 驱动按 CRC 丢弃）")

    # 交织：坏帧 + 好帧，解析器必须只交出 2 帧且都能再校验
    stream = build_frame(ID_CHASSIS_DATA, good_chassis)[0] + b"\x7d\x16\x01\x02\x03\x7e" \
        + build_frame(ID_CHASSIS_DATA, good_chassis)[0]
    frames = FrameParser().feed(stream)
    check("resync_after_garbage", len(frames) == 3 and [len(f) for f in frames] == [29, 6, 29],
          f"帧长={[len(f) for f in frames]} (坏帧 6B 被长度校验挡下，后续帧照常)")
    split = FrameParser()
    check("resync_split", split.feed(esc_escaped[:3]) == [] and len(split.feed(esc_escaped[3:])) == 1,
          "分片喂入（3B + 剩余）也能拼回整帧")

    print("=" * 96)
    print("fake_ecu.py 纯协议自检 / SELFTEST（不需要 ROS / 串口）")
    print("-" * 96)
    print("推导的帧格式（仓库出处见文件头）:")
    print(f"  0x11 RefereeData    载荷 {PAYLOAD_SIZE[ID_REFEREE]}B -> 帧 {FRAME_SIZE[ID_REFEREE]}B")
    print(f"  0x16 ChassisData    载荷 {PAYLOAD_SIZE[ID_CHASSIS_DATA]}B -> 帧 {FRAME_SIZE[ID_CHASSIS_DATA]}B")
    print(f"  0x20 ChassisControl 载荷 {PAYLOAD_SIZE[ID_CHASSIS_CONTROL]}B -> 帧 {FRAME_SIZE[ID_CHASSIS_CONTROL]}B"
          f"  示例 raw={raw.hex(' ')}")
    print("-" * 96)
    print("\n".join(out))
    print("-" * 96)
    print(f"RESULT: {'PASS' if ok else 'FAIL'}  asserts={sum(1 for l in out if ' PASS ' in l)}/{len(out)}")
    print("=" * 96)
    return 0 if ok else 1


# --------------------------------------------------------------------------- #
def build_parser():
    p = argparse.ArgumentParser(
        prog="fake_ecu.py",
        description="假 ECU：发 0x11/0x16 帧、收 0x20 控制帧做校验/对表/看门狗验证（无硬件台架对端）",
        epilog=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    g = p.add_argument_group("链路")
    g.add_argument("--link", default="/tmp/ttyFAKE_ECU", help="pty 路径（默认 /tmp/ttyFAKE_ECU）")
    g.add_argument("--baud", type=int, default=1000000, help="波特率，pty 上无效但需与驱动一致（默认 1000000）")
    g.add_argument("--link-timeout", type=float, default=10.0, help="等链路出现的秒数（默认 10）")

    g = p.add_argument_group("发送（ECU -> 驱动）")
    g.add_argument("--referee-hz", type=float, default=10.0, help="0x11 裁判系统帧速率 Hz（默认 10）")
    g.add_argument("--chassis-hz", type=float, default=50.0, help="0x16 底盘数据帧速率 Hz（默认 50）")
    g.add_argument("--chassis-speed", type=float, default=300.0, help="合成底盘速度幅度 mm/s（默认 300）")
    g.add_argument("--chassis-cycle", type=float, default=4.0, help="合成底盘速度周期 s（默认 4）")
    g.add_argument("--chassis-echo", action="store_true",
                   help="底盘数据回显最后一条速度指令（乘 --chassis-echo-scale），默认走合成正弦")
    g.add_argument("--chassis-echo-scale", type=float, default=1.0, help="回显缩放（默认 1.0）")

    g = p.add_argument_group("故障注入")
    g.add_argument("--corrupt-every", type=int, default=0, metavar="N",
                   help="每 N 条 0x16 帧里 1 条载荷改哨兵值（帧长/帧尾不变 -> CRC 必须不符），0=关")
    g.add_argument("--garbage-every", type=int, default=0, metavar="N",
                   help="每 N 条 0x16 帧里发 1 条长度不足的畸形帧，0=关")
    g.add_argument("--poison-vx", type=float, default=123456.0,
                   help="坏帧哨兵值（用于证明非法帧没变成 /chassis_data 数据，默认 123456）")
    g.add_argument("--pause-after", type=float, default=None, metavar="SEC",
                   help="T0 后 SEC 秒开始停发 ECU 侧帧（考驱动读线程）")
    g.add_argument("--pause-for", type=float, default=0.0, help="停发时长 s（配合 --pause-after）")
    g.add_argument("--stop-velocity-after", type=float, default=None, metavar="SEC",
                   help="T0 后 SEC 秒停止发布 /sentry/velocity_command（考 velocity_timeout）")
    g.add_argument("--stop-posture-after", type=float, default=None, metavar="SEC",
                   help="T0 后 SEC 秒停止发布 /sentry/posture_command（考 posture_timeout）")

    g = p.add_argument_group("指令侧（rclpy）")
    g.add_argument("--no-cmd", action="store_true", help="完全不发布速度/姿态指令")
    g.add_argument("--cmd-hz", type=float, default=20.0, help="指令发布频率 Hz（默认 20）")
    g.add_argument("--cmd-hold", type=float, default=1.0, help="sign-check 下每组预设保持秒数（默认 1.0）")
    g.add_argument("--vx", type=float, default=None, help="固定 vx（mm/s），默认三组预设轮换")
    g.add_argument("--vy", type=float, default=None, help="固定 vy（mm/s）")
    g.add_argument("--wz", type=float, default=None, help="固定 wz（deg/s）")
    g.add_argument("--posture", type=int, default=1, help="姿态指令值（默认 1=得分姿态）")
    g.add_argument("--velocity-topic", default="/sentry/velocity_command")
    g.add_argument("--posture-topic", default="/sentry/posture_command")
    g.add_argument("--chassis-topic", default="/chassis_data")

    g = p.add_argument_group("断言")
    g.add_argument("--min-rate-hz", type=float, default=20.0,
                   help="驱动->ECU 帧率下限（稳态窗口），默认 20")
    g.add_argument("--rate-warmup", type=float, default=1.0, help="帧率窗口从 T0+该秒数开始（默认 1.0）")
    g.add_argument("--rate-window", type=float, default=2.0, help="日志里的滑动帧率窗口 s（默认 2.0）")
    g.add_argument("--first-frame-timeout", type=float, default=8.0, help="等驱动第一帧的超时 s（默认 8）")
    g.add_argument("--sign-check", action="store_true",
                   help="订阅速度/姿态指令，逐帧断言串口帧 == 最后下发的指令（对表证据）")
    g.add_argument("--tol", type=float, default=1e-6,
                   help="对表容差（mm/s、deg/s；默认 1e-6；取负值必然 FAIL，可用来验证断言真的会失败）")
    g.add_argument("--tol-window", type=float, default=0.15,
                   help="停发指令后停止对表的宽限 s（默认 0.15）")
    g.add_argument("--cmd-grace", type=float, default=2.0,
                   help="驱动吃到第一条指令前的启动宽限 s（DDS 发现耗时；超期仍对不上即判 FAIL，默认 2.0）")
    g.add_argument("--check-chassis", action="store_true",
                   help="订阅 /chassis_data：断言底盘数据仍在发布且哨兵坏帧没变成数据")
    g.add_argument("--min-chassis-hz", type=float, default=5.0, help="底盘数据末段速率下限（默认 5）")
    g.add_argument("--chassis-window", type=float, default=2.0, help="底盘速率断言看的末段窗口 s（默认 2）")
    g.add_argument("--velocity-timeout", type=float, default=0.2,
                   help="驱动的 velocity_timeout，用于算零速出现时限（默认 0.2）")
    g.add_argument("--posture-timeout", type=float, default=1.0,
                   help="驱动的 posture_timeout（默认 1.0）")
    g.add_argument("--expect-posture", type=int, default=3,
                   help="姿态超时后应回到的姿态（默认 3=移动姿态）")
    g.add_argument("--watchdog-margin", type=float, default=0.5, help="看门狗时限余量 s（默认 0.5）")

    g = p.add_argument_group("其它")
    g.add_argument("--duration", type=float, default=10.0, help="总运行时长 s（默认 10）")
    g.add_argument("--log-hz", type=float, default=1.0, help="周期日志频率 Hz（默认 1，0=关闭）")
    g.add_argument("--selftest", action="store_true", help="只跑纯协议自检（不需要 ROS/串口）后退出")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.selftest:
        return run_selftest()
    if args.duration <= 0:
        print("--duration 必须 > 0", file=sys.stderr)
        return 2
    if args.referee_hz <= 0 or args.chassis_hz <= 0:
        print("--referee-hz/--chassis-hz 必须 > 0", file=sys.stderr)
        return 2
    if args.pause_after is not None and args.pause_for <= 0:
        print("--pause-after 需要配合 > 0 的 --pause-for", file=sys.stderr)
        return 2

    ecu = FakeEcu(args)
    try:
        return ecu.run()
    except KeyboardInterrupt:
        print("\n[ecu] Ctrl-C：收尾", flush=True)
        return 130
    finally:
        ecu.close()


if __name__ == "__main__":
    sys.exit(main())
