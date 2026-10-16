#!/usr/bin/env bash
# -*- coding: utf-8 -*-
# =============================================================================
# bench_uart.sh —— 一条命令跑通「串口链路台架」：uart_driver_node + 假 ECU（tools/fake_ecu.py）
#
# 用法:
#     bash tools/bench_uart.sh [传给 fake_ecu.py 的额外参数...]
#
#     # 例：调高驱动->ECU 帧率下限（3=30Hz），或把某个场景的时长加长
#     bash tools/bench_uart.sh --min-rate-hz 45
#     # 例：故意让断言失败，验证台架真的会报错（退出码非 0）
#     bash tools/bench_uart.sh --min-rate-hz 5000
#     bash tools/bench_uart.sh --tol -1 --sign-check     # 负容差 -> sign_check 必然 FAIL
#
# 它做什么:
#   1. 建一对 pty（有 socat 就用 `socat -d -d pty,raw,echo=0,link=...`，没有就用等价的纯 python 桥）
#      —— 本机实测没有 socat，所以默认走 python 桥，语义与 socat raw,echo=0 一致。
#   2. 起真实 uart_driver_node（仓库 config/driver_config.yaml + -p serial_name:=<假端>）。
#   3. 用 tools/fake_ecu.py 依次跑 5 个脚本化场景，每个场景自己断言并打印 RESULT 行。
#   4. 汇总成 PASS/FAIL 表 + 关键日志摘录，最后清理（trap，含 Ctrl-C）并对失败返回非 0。
#
# 场景:
#   a 正常链路     帧尺寸/转义/CRC 正确、值与指令一致(sign-check)、驱动->ECU 帧率 >= 阈值
#   b 注入坏帧     坏帧/畸形帧被驱动丢弃并计数，合法帧继续正常，非法帧没变成 /chassis_data 数据
#   c 底盘数据停发 驱动不崩：暂停期间照常发控制帧，恢复后继续正常收发
#   d 速度指令停发 velocity_timeout(0.2s) 后驱动开始发零速
#   e 姿态指令停发 posture_timeout(1.0s) 后驱动回到移动姿态 3
#
# 隔离: 全程 ROS_DOMAIN_ID=42 + FASTDDS_BUILTIN_TRANSPORTS=SHM（本机 DDS 走 UDP 不可用），
#       只杀本脚本自己起的进程（setsid 进程组 + 带本脚本链路路径的 [x] 自保护 pkill 模式）。
# =============================================================================
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

usage() {
    cat <<'EOF'
用法: bash tools/bench_uart.sh [传给 fake_ecu.py 的额外参数...]

  不带参数       : 跑全部 5 个场景（正常链路 / 坏帧注入 / 底盘停发 / 速度指令停发 / 姿态指令停发）
  --min-rate-hz N: 覆盖驱动->ECU 帧率下限（默认每场景 30 Hz）
  --tol F        : 覆盖对表容差（默认 1e-6；取负值必然 FAIL，用于验证台架真的会报错）
  其它任何 fake_ecu.py 的参数都会被附加到每个场景（场景自己的参数先给，用户参数随后覆盖）

  例:
    bash tools/bench_uart.sh                          # 正常跑，期望 5/5 PASS、退出码 0
    bash tools/bench_uart.sh --min-rate-hz 5000        # 故意失败：不可达帧率下限 -> 退出码 1
    bash tools/bench_uart.sh --tol -1 --sign-check     # 故意失败：对表容差为负 -> 退出码 1
    python3 tools/fake_ecu.py --help                   # 假 ECU 的全部参数
    python3 tools/fake_ecu.py --selftest               # 纯协议自检（不需要 ROS / 串口）

隔离: 本脚本固定 ROS_DOMAIN_ID=42 + FASTDDS_BUILTIN_TRANSPORTS=SHM，只清理自己起的进程。
EOF
}

case "${1:-}" in
    -h|--help) usage; exit 0 ;;
esac

DOMAIN=42
LINK_A="/tmp/ttyBench_driver"      # 驱动打开的一端
LINK_B="/tmp/ttyBench_ecu"         # 假 ECU 打开的一端
TMP="$(mktemp -d /tmp/bench_uart.XXXXXX)"
DRIVER_LOG="$TMP/driver.log"
BRIDGE_LOG="$TMP/bridge.log"
# 驱动的日志带 ANSI 颜色码（open /tmp/... 之间会插入 \x1b[32m），所有读取都先剥色
driver_plain() { sed -E $'s/\x1b\[[0-9;]*m//g' "$DRIVER_LOG" 2>/dev/null; }
EXTRA=("$@")
BRIDGE_PID=""
DRIVER_PID=""
FAILED=0
declare -a ROW_KEYS=() ROW_DESC=() ROW_VERDICT=() ROW_EVID=() ROW_LOG=()

export ROS_DOMAIN_ID="$DOMAIN"
export FASTDDS_BUILTIN_TRANSPORTS=SHM
export RCUTILS_LOGGING_BUFFERED_STREAM=0     # 驱动日志及时落盘

# --------------------------------------------------------------------------- #
cleanup() {
    rc=$?
    trap - EXIT INT TERM
    # 只杀自己起的进程：先按进程组，再按「本脚本链路路径」自保护模式兜底
    if [ -n "$DRIVER_PID" ]; then
        kill -TERM "-$DRIVER_PID" 2>/dev/null
        sleep 0.3
        kill -KILL "-$DRIVER_PID" 2>/dev/null
    fi
    if [ -n "$BRIDGE_PID" ]; then
        kill -TERM "-$BRIDGE_PID" 2>/dev/null
        sleep 0.2
        kill -KILL "-$BRIDGE_PID" 2>/dev/null
    fi
    pkill -TERM -f "[f]ake_ecu.py --link $LINK_B" 2>/dev/null
    pkill -KILL -f "[u]art_driver_node.*ttyBench_driver" 2>/dev/null
    pkill -KILL -f "[p]ty_bridge.py $LINK_A" 2>/dev/null
    pkill -KILL -f "[s]ocat -d -d pty,raw,echo=0,link=$LINK_A" 2>/dev/null
    rm -f "$LINK_A" "$LINK_B"
    exit "$rc"
}
trap cleanup EXIT INT TERM

# --------------------------------------------------------------------------- #
echo "=============================================================="
echo "串口链路台架 / UART BENCH    ROS_DOMAIN_ID=$DOMAIN  transport=SHM"
echo "  workspace : $ROOT"
echo "  设备对    : $LINK_A (驱动) <-> $LINK_B (假 ECU)"
echo "  临时目录  : $TMP"
if [ "${#EXTRA[@]}" -gt 0 ]; then
    echo "  额外参数  : ${EXTRA[*]}（附加到每个场景，覆盖场景默认值）"
fi
echo "  用法      : bash tools/bench_uart.sh [传给 fake_ecu.py 的额外参数...]（--help 看说明）"
echo "=============================================================="

if [ ! -f /opt/ros/kilted/setup.bash ]; then
    echo "找不到 /opt/ros/kilted/setup.bash"; exit 2
fi
if [ ! -f install/setup.bash ]; then
    echo "找不到 install/setup.bash：请先 colcon build 后再跑（本脚本不编译）"; exit 2
fi
# shellcheck disable=SC1091
set +u                                   # ROS 的 setup.bash 会引用未定义变量，必须先关 nounset
source /opt/ros/kilted/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u
command -v ros2 >/dev/null || { echo "ros2 不在 PATH"; exit 2; }

# --------------------------------------------------------------------------- #
# 1) 设备对
# --------------------------------------------------------------------------- #
rm -f "$LINK_A" "$LINK_B"
if command -v socat >/dev/null 2>&1; then
    TRANSPORT="socat -d -d pty,raw,echo=0,link=..."
    setsid socat -d -d pty,raw,echo=0,link="$LINK_A" pty,raw,echo=0,link="$LINK_B" >"$BRIDGE_LOG" 2>&1 &
else
    TRANSPORT="纯 python pty 桥（本机无 socat，语义等价于 socat raw,echo=0）"
    cat >"$TMP/pty_bridge.py" <<'PYEOF'
# 与 `socat -d -d pty,raw,echo=0,link=A pty,raw,echo=0,link=B` 等价的纯 python 实现：
# 两对 pty 的 master 端由本进程持有，slave 端以符号链接暴露给驱动 / 假 ECU，双向透传字节。
import os, pty, select, sys, tty

link_a, link_b = sys.argv[1], sys.argv[2]
m1, s1 = pty.openpty()
m2, s2 = pty.openpty()
for fd in (m1, m2):
    tty.setraw(fd)                      # 关 echo / 行变换，等价 socat 的 raw,echo=0
for p in (link_a, link_b):
    try:
        os.unlink(p)
    except FileNotFoundError:
        pass
os.symlink(os.ttyname(s1), link_a)
os.symlink(os.ttyname(s2), link_b)
os.close(s1)
os.close(s2)                            # 让打开方成为 slave 的唯一持有者
print("[pty-bridge] %s <-> %s" % (link_a, link_b), flush=True)

while True:
    try:
        ready, _, _ = select.select([m1, m2], [], [], 1.0)
    except (OSError, InterruptedError):
        continue
    for fd in ready:
        try:
            data = os.read(fd, 4096)
        except OSError:                 # 对端还没打开 / 已关闭：跳过（pty 会报 EIO）
            continue
        if not data:
            continue
        try:
            os.write(m2 if fd == m1 else m1, data)
        except OSError:
            pass
PYEOF
    setsid python3 "$TMP/pty_bridge.py" "$LINK_A" "$LINK_B" >"$BRIDGE_LOG" 2>&1 &
fi
BRIDGE_PID=$!
disown "$BRIDGE_PID" 2>/dev/null || true

for _ in $(seq 1 50); do
    [ -e "$LINK_A" ] && [ -e "$LINK_B" ] && break
    sleep 0.1
done
if [ ! -e "$LINK_A" ] || [ ! -e "$LINK_B" ]; then
    echo "设备对创建失败: $LINK_A / $LINK_B"; cat "$BRIDGE_LOG"; exit 2
fi
echo "设备对就绪: $TRANSPORT"
echo "  $LINK_A -> $(readlink -f "$LINK_A")    $LINK_B -> $(readlink -f "$LINK_B")"

# --------------------------------------------------------------------------- #
# 2) 真实驱动节点
# --------------------------------------------------------------------------- #
CFG="install/uart_driver/share/uart_driver/config/driver_config.yaml"
setsid ros2 run uart_driver uart_driver_node --ros-args \
    --params-file "$CFG" \
    -p serial_name:="$LINK_A" \
    -p baud_rate:=1000000 \
    -p read_buffer_size:=60 \
    -p possible_ids:="[17, 22]" \
    -p velocity_command_topic:=/sentry/velocity_command \
    -p posture_command_topic:=/sentry/posture_command \
    -p send_rate_hz:=50.0 \
    -p velocity_timeout:=0.2 \
    -p posture_timeout:=1.0 \
    -p default_posture:=3 >"$DRIVER_LOG" 2>&1 &
DRIVER_PID=$!
disown "$DRIVER_PID" 2>/dev/null || true

READY=0
for _ in $(seq 1 100); do
    if driver_plain | grep -q "open $LINK_A in"; then READY=1; break; fi
    sleep 0.2
done
if [ "$READY" != "1" ]; then
    echo "uart_driver_node 没能打开 $LINK_A（等 20s）："
    driver_plain | tail -20
    exit 2
fi
echo "驱动就绪: $(driver_plain | grep -m1 "open $LINK_A in")"

# --------------------------------------------------------------------------- #
# 3) 场景
# --------------------------------------------------------------------------- #
run_one() {
    local key="$1" desc="$2" dur="$3" evgrep="$4"; shift 4
    local log="$TMP/fake_ecu_${key}.log"
    local cmd=(python3 tools/fake_ecu.py --link "$LINK_B")
    cmd+=("$@")
    if [ "${#EXTRA[@]}" -gt 0 ]; then cmd+=("${EXTRA[@]}"); fi
    cmd+=(--duration "$dur")

    echo ""
    echo "--------------------------------------------------------------"
    echo "场景 $key: $desc"
    echo "  \$ ${cmd[*]}"
    echo "--------------------------------------------------------------"
    "${cmd[@]}" >"$log" 2>&1
    local rc=$?

    local res verdict ev
    res="$(grep -m1 '^RESULT:' "$log" 2>/dev/null)"
    if [ -z "$res" ]; then
        verdict="FAIL"
        res="RESULT 行缺失（进程异常退出 rc=$rc）"
    elif [ "$rc" -eq 0 ]; then
        verdict="PASS"
    else
        verdict="FAIL"
    fi
    ev="$(grep -E "$evgrep" "$log" 2>/dev/null | head -1)"
    if [ "$verdict" = "FAIL" ]; then
        fev="$(grep -E '^ASSERT .*FAIL' "$log" 2>/dev/null | head -1)"
        [ -n "$fev" ] && ev="$fev"
    fi
    [ -n "$ev" ] || ev="(没有匹配到证据行 '$evgrep')"

    # 打印本场景的关键日志（周期行只留最后一行，其余留断言/事件）
    grep -E "^\[ecu|^ASSERT|^RESULT:|^fake ECU|^RX |^TX |^对表|^/chassis|^下发|^  EVENT|^  例:" "$log" \
        | grep -vE "rx=[0-9]+ \(" | sed 's/^/  | /'
    grep -E "rx=[0-9]+ \(" "$log" | tail -1 | sed 's/^/  | (周期行,尾) /'
    echo "  -> rc=$rc  $verdict"
    echo "  驱动侧: $(driver_evidence "$key")"

    ROW_KEYS+=("$key"); ROW_DESC+=("$desc"); ROW_VERDICT+=("$verdict")
    ROW_EVID+=("$ev"); ROW_LOG+=("$log")
    [ "$verdict" = "PASS" ] || FAILED=1
}

driver_evidence() {
    case "$1" in
        a) local n; n="$(driver_plain | grep -c "invalid .* frame dropped")"
           echo "驱动无非法帧告警（invalid 行数=$n）" ;;
        b) driver_plain | grep -m2 "invalid .* frame dropped" | tr '\n' ' ' ;;
        c) local ns rp; ns="$(driver_plain | grep -c "no serial data")"; rp="$(driver_plain | grep -c "Successfully reopened")"
           echo "停发期间驱动未崩：no_serial_data 行数=$ns，串口重开次数=$rp" ;;
        d) driver_plain | grep -m1 "velocity command timeout" ;;
        e) echo "姿态只走串口帧，证据见上行 EVENT posture_move_first（驱动无姿态日志）" ;;
        *) echo "-" ;;
    esac
}

run_one a "正常链路：帧尺寸/转义/CRC 正确、值一致(sign-check)、帧率达标" 8 \
    "ASSERT (link_rate|rx_protocol|escape_path|sign_check|chassis_rate)" \
    --sign-check --check-chassis --min-rate-hz 30 --min-chassis-hz 20

run_one b "注入坏帧：驱动丢弃并计数、合法帧继续正常、非法帧不得变成数据" 10 \
    "ASSERT no_fake_data" \
    --sign-check --check-chassis --corrupt-every 10 --garbage-every 25 --min-rate-hz 30

run_one c "底盘数据停发 4s：驱动不崩、恢复后继续正常收发" 11 \
    "ASSERT pause_(injected|resume)" \
    --check-chassis --pause-after 3 --pause-for 4 --min-rate-hz 30

run_one d "速度指令停发：velocity_timeout=0.2s 后驱动发零速" 8 \
    "EVENT velocity_zero_first|ASSERT velocity_watchdog" \
    --stop-velocity-after 3 --velocity-timeout 0.2 --min-rate-hz 30

run_one e "姿态指令停发：posture_timeout=1.0s 后驱动回移动姿态 3" 8 \
    "EVENT posture_move_first|ASSERT posture_watchdog" \
    --stop-posture-after 3 --posture-timeout 1.0 --expect-posture 3 --sign-check --min-rate-hz 30

# --------------------------------------------------------------------------- #
# 4) 汇总表
# --------------------------------------------------------------------------- #
echo ""
echo "=============================================================="
echo "台架结果 / BENCH RESULT（每个场景由 fake_ecu.py 自己断言，任一 FAIL 则退出码非 0）"
echo "=============================================================="
: >"$TMP/table.tsv"
for i in "${!ROW_KEYS[@]}"; do
    printf '%s\t%s\t%s\t%s\n' "${ROW_KEYS[$i]}" "${ROW_DESC[$i]}" "${ROW_VERDICT[$i]}" \
        "$(echo "${ROW_EVID[$i]}" | sed -E 's/^\[ecu +[0-9.]+s\] //')" >>"$TMP/table.tsv"
done
python3 - "$TMP/table.tsv" <<'PYEOF'
# 表格按「显示宽度」对齐：CJK 字符按 2 列算（中文描述用 printf 的字节宽度会错位）
import sys, unicodedata

def w(s):
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)

def pad(s, n):
    return s + " " * max(0, n - w(s))

rows = [l.rstrip("\n").split("\t") for l in open(sys.argv[1], encoding="utf-8") if l.strip()]
head = ["场景", "内容", "结果", "关键证据"]
n0 = max([w(head[0])] + [w(r[0]) for r in rows]) if rows else w(head[0])
n1 = max([w(head[1])] + [w(r[1]) for r in rows]) if rows else w(head[1])
n2 = max([w(head[2])] + [w(r[2]) for r in rows]) if rows else w(head[2])
line = lambda c: "  ".join((pad(c[0], n0), pad(c[1], n1), pad(c[2], n2), c[3]))
print(line(head))
print("-" * (n0 + n1 + n2 + 8))
for r in rows:
    print(line(r))
PYEOF
echo "--------------------------------------------------------------"
echo "各场景 RESULT 行:"
for i in "${!ROW_KEYS[@]}"; do
    printf "  [%s] %s\n" "${ROW_KEYS[$i]}" "$(grep -m1 '^RESULT:' "${ROW_LOG[$i]}" 2>/dev/null)"
done
N_FAIL=$(for i in "${!ROW_VERDICT[@]}"; do echo "${ROW_VERDICT[$i]}"; done | grep -c FAIL)
if [ "$N_FAIL" -eq 0 ]; then
    echo ""
    echo "台架总结: PASS（${#ROW_KEYS[@]}/${#ROW_KEYS[@]} 个场景通过，退出码 0）"
    echo "日志: $TMP   （驱动日志 $DRIVER_LOG）"
    echo "=============================================================="
else
    echo ""
    echo "台架总结: FAIL（$N_FAIL/${#ROW_KEYS[@]} 个场景失败，退出码 1）—— 细节见上面各场景输出"
    echo "日志: $TMP   （驱动日志 $DRIVER_LOG）"
    echo "=============================================================="
fi
exit "$FAILED"
