#!/usr/bin/env bash
# 小电脑环境一键检测（P6.2 / 4.md §3）—— 到场地后第一件事，5 秒出结论。
#
#   bash tools/check_env.sh                 # 全量检测（系统 + ROS + 包 + 依赖 + 设备 + 标定文件）
#   bash tools/check_env.sh --no-devices    # 开发机/台架：跳过 odin USB 与串口两项
#   bash tools/check_env.sh --serial /dev/ttyACM0 --workspace ~/SentryMaster2026
#   bash tools/check_env.sh --strict        # 有 WARN 也算失败（出发前在驻地跑一遍，要求全绿）
#
# 退出码：0 = 无 FAIL（--strict 时要求无 WARN）；1 = 有 FAIL；2 = 参数错误
# 只读检测，不改任何配置、不启动任何节点。
set -uo pipefail

WORKSPACE="$(cd "$(dirname "$0")/.." && pwd)"
SERIAL=""
CHECK_DEVICES=1
CHECK_ODIN=1
CHECK_ROS=1
STRICT=0
ROS_SETUP="/opt/ros/kilted/setup.bash"

while [ $# -gt 0 ]; do
    case "$1" in
        --workspace)   WORKSPACE="$2"; shift 2 ;;
        --serial)      SERIAL="$2"; shift 2 ;;
        --no-devices)  CHECK_DEVICES=0; shift ;;
        --no-odin)     CHECK_ODIN=0; shift ;;
        --no-ros)      CHECK_ROS=0; shift ;;
        --strict)      STRICT=1; shift ;;
        -h|--help)
            sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *) echo "未知参数: $1（--help 查看用法）" >&2; exit 2 ;;
    esac
done

PASS=0; WARN=0; FAIL=0
ok()   { PASS=$((PASS+1)); printf '  \033[32m[PASS]\033[0m %-22s %s\n' "$1" "$2"; }
warn() { WARN=$((WARN+1)); printf '  \033[33m[WARN]\033[0m %-22s %s\n' "$1" "$2"; }
bad()  { FAIL=$((FAIL+1)); printf '  \033[31m[FAIL]\033[0m %-22s %s\n' "$1" "$2"; }
sec()  { printf '\n\033[36m== %s ==\033[0m\n' "$1"; }

echo "=============================================================="
echo " SentryMaster2026 环境自检   工作空间: $WORKSPACE   时间: $(date '+%F %T')"
echo "=============================================================="

# ---------------------------------------------------------------- 1. 系统资源
sec "1. 系统与资源"
OS_NAME="$( . /etc/os-release 2>/dev/null && echo "$PRETTY_NAME" )"
CORES="$(nproc 2>/dev/null || echo '?')"
MEM_GB="$(awk '/MemTotal/{printf "%.1f", $2/1048576}' /proc/meminfo 2>/dev/null)"
echo "  host: $(hostname)  os: ${OS_NAME:-?}  kernel: $(uname -r)  cores: $CORES  mem: ${MEM_GB}GB"
if [ -n "$CORES" ] && [ "$CORES" != "?" ] && [ "$CORES" -ge 4 ]; then
    ok "cpu" "${CORES} 核（编译建议 bash scripts/build.sh -s 单线程，避免小电脑内存打爆）"
else
    warn "cpu" "${CORES} 核：编译请务必用 'bash scripts/build.sh -s'，运行期别同时跑 RViz"
fi

for DIR in "$WORKSPACE" /tmp; do
    FREE_GB="$(df -Pk "$DIR" 2>/dev/null | awk 'NR==2{printf "%.1f", $4/1048576}')"
    if [ -n "${FREE_GB:-}" ] && awk "BEGIN{exit !($FREE_GB >= 2)}"; then
        ok "disk:$DIR" "剩余 ${FREE_GB} GB（>=2 GB：够录一整局 bag + 日志）"
    else
        bad "disk:$DIR" "剩余 ${FREE_GB:-?} GB < 2 GB —— 先清 bag/日志，否则录包会写满"
    fi
done

if command -v timedatectl > /dev/null 2>&1; then
    SYNC="$(timedatectl show -p NTPSynchronized --value 2>/dev/null)"
    if [ "$SYNC" = "yes" ]; then
        ok "time-sync" "NTP 已同步（odin 配了 use_host_ros_time: 2，主机时间会进时间戳）"
    else
        warn "time-sync" "未同步（SYNC=${SYNC:-未知}）：离线场地上电后 apt 时间会偏，bag/tf 时间戳跟着偏"
    fi
else
    warn "time-sync" "无 timedatectl（非 systemd 环境）：上电后手工确认日期时间"
fi

# ---------------------------------------------------------------- 2. ROS 环境
sec "2. ROS 环境与工作空间"
if [ -f "$ROS_SETUP" ]; then
    ok "ros-setup" "$ROS_SETUP 存在"
    # ROS 的 setup.bash 不是 set -u 干净的（会读未定义变量），source 前后临时关掉 -u
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    ok "ros-distro" "ROS_DISTRO=${ROS_DISTRO:-?}  $(ros2 --version 2>/dev/null | head -1)"
    set -u
else
    bad "ros-setup" "找不到 $ROS_SETUP —— 车上 ROS 没装或发行版不是 kilted"
fi

if [ -f "$WORKSPACE/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$WORKSPACE/install/setup.bash"
    set -u
    ok "ws-build" "install/setup.bash 存在（已 source）"
else
    bad "ws-build" "缺 $WORKSPACE/install/setup.bash —— 先 cd scripts && bash build.sh -s"
fi

if [ "$CHECK_ROS" = "1" ] && command -v ros2 > /dev/null 2>&1; then
    MISSING=""
    for PKG in rm_msgs uart_driver sentry_planner sentry_decision_sys sentry_launch odin_ros_driver; do
        ros2 pkg prefix "$PKG" > /dev/null 2>&1 || MISSING="$MISSING $PKG"
    done
    if [ -z "$MISSING" ]; then
        ok "ros-pkgs" "6 个包全部可解析（rm_msgs/uart_driver/sentry_planner/sentry_decision_sys/sentry_launch/odin_ros_driver）"
    else
        bad "ros-pkgs" "无法解析:$MISSING —— 重新 source install/setup.bash 或重新编译"
    fi

    MISSING_EXE=""
    for SPEC in "sentry_planner sentry_planner_node" "sentry_decision_sys sentry_decision_node" "uart_driver uart_driver_node"; do
        set -- $SPEC
        ros2 pkg executables "$1" 2>/dev/null | grep -q "$2" || MISSING_EXE="$MISSING_EXE $2"
    done
    if [ -z "$MISSING_EXE" ]; then
        ok "ros-executables" "sentry_planner_node / sentry_decision_node / uart_driver_node 都在"
    else
        bad "ros-executables" "缺可执行:$MISSING_EXE —— 编译没跑完或包没装上"
    fi

    STRAY="$(timeout 10 ros2 node list 2>/dev/null | grep -c . )"
    if [ "${STRAY:-0}" -eq 0 ]; then
        ok "ros-graph" "当前没有残留节点"
    else
        warn "ros-graph" "已有 $STRAY 个节点在跑（$(timeout 5 ros2 node list 2>/dev/null | tr '\n' ' ')）—— 重复起同名节点会抢串口/话题"
    fi
fi

# ---------------------------------------------------------------- 3. python 依赖
sec "3. Python 依赖（工具链 / 节点）"
MISSING_PY="$(python3 - <<'PY' 2>/dev/null
import importlib
need = ["numpy", "yaml", "rclpy", "tf2_ros"]
opt = ["scipy", "PIL"]
miss = [m for m in need if not importlib.util.find_spec(m)]
opt_miss = [m for m in opt if not importlib.util.find_spec(m)]
print(",".join(miss) + "|" + ",".join(opt_miss))
PY
)"
REQ_MISS="${MISSING_PY%%|*}"; OPT_MISS="${MISSING_PY##*|}"
if [ -z "$REQ_MISS" ]; then
    ok "py-req" "numpy / yaml / rclpy / tf2_ros 均可 import"
else
    bad "py-req" "缺:$REQ_MISS —— 节点或工具会直接崩，先补齐"
fi
if [ -z "$OPT_MISS" ]; then
    ok "py-opt" "scipy / PIL 可用（地图距离变换走 scipy 精确解、RViz 截图可用）"
else
    warn "py-opt" "缺:$OPT_MISS（可选：scipy 缺席时 build_nav_map 退回自实现 EDT，PIL 只影响截图）"
fi

# ---------------------------------------------------------------- 4. 标定/参数文件
sec "4. 标定与参数文件"
map_bin="$WORKSPACE/src/sentry_planner/map/701_model/nav_map.bin"
if [ -f "$map_bin" ]; then
    ok "nav_map" "$(du -h "$map_bin" | cut -f1)  $(ls -l "$map_bin" | awk '{print $NF}')"
else
    bad "nav_map" "缺 nav_map.bin —— 跑 python3 tools/build_nav_map.py"
fi

GRAPH="$WORKSPACE/src/sentry_planner/config/waypoint_graph.yaml"
if [ -f "$GRAPH" ]; then
    NODES="$(python3 -c "import yaml,sys;d=yaml.safe_load(open('$GRAPH')) or {};print(len(d.get('nodes') or []))" 2>/dev/null)"
    if [ "${NODES:-0}" -gt 0 ]; then
        ok "waypoint_graph" "${NODES} 个节点（导航可以规划）"
    else
        warn "waypoint_graph" "0 个节点 —— 还没打点：导航会打 [fault] 拒绝运动（require_map=true），现场先跑 tools/mark_points.py"
    fi
else
    warn "waypoint_graph" "文件不存在 —— 同上"
fi

TABLE="$WORKSPACE/src/sentry_decision_sys/config/task_table.yaml"
if [ -f "$TABLE" ]; then
    # 只检查「被 phases 引用」的点位：本策略只做 ①②⑤，③④ 留着作参考但不需要打点
    STATS="$(python3 - "$TABLE" <<'PY' 2>/dev/null
import math, sys, yaml

d = yaml.safe_load(open(sys.argv[1])) or {}
pts = d.get("points") or {}
used = set()
for ph in d.get("phases") or []:
    for key in ("point", "pre_point"):
        if ph.get(key):
            used.add(ph[key])
need = [k for k in sorted(used) if k in pts]
def is_nan(value):
    v = (pts.get(value) or {}).get("x")
    return v is None or (isinstance(v, float) and math.isnan(v))
missing = [k for k in need if is_nan(k)]
unused = [k for k in pts if k not in used]
print(f"{len(need)}|{len(missing)}|{','.join(missing)}|{','.join(unused)}")
PY
)"
    TOTAL="${STATS%%|*}"; REST="${STATS#*|}"
    NAN_N="${REST%%|*}"; REST="${REST#*|}"
    NAN_LIST="${REST%%|*}"; UNUSED="${REST#*|}"
    if [ "${NAN_N:-1}" -eq 0 ]; then
        ok "task_table" "阶段引用的 ${TOTAL} 个点位已全部打点${UNUSED:+（未使用、不用打点: $UNUSED）}"
    else
        warn "task_table" "${NAN_N}/${TOTAL} 个**被阶段引用**的点位未打点（.nan）:$NAN_LIST —— 对应阶段会被跳过，现场必须打点${UNUSED:+（未使用: $UNUSED）}"
    fi
else
    bad "task_table" "缺 task_table.yaml"
fi

ALIGN="$WORKSPACE/src/sentry_planner/config/map_alignment.yaml"
if [ -f "$ALIGN" ]; then
    RMS="$(python3 -c "import yaml;d=yaml.safe_load(open('$ALIGN')) or {};print(d.get('rms_m'), d.get('n'))" 2>/dev/null)"
    ok "map_alignment" "rms/n = $RMS"
else
    warn "map_alignment" "未做地图对齐（纯里程计方案下非必需，但打点校验与 RViz 叠加要用）——现场 §5 做"
fi

LIMITS="$WORKSPACE/src/sentry_planner/config/speed_limits.yaml"
if [ -f "$LIMITS" ]; then
    if grep -qa "no power data" "$LIMITS"; then
        warn "speed_limits" "存在但**未经真车标定**（头部写着 no power data，是保守占位）——现场 §7 覆盖它"
    else
        ok "speed_limits" "已标定: $(grep -E 'max_vx|max_wz' "$LIMITS" | tr -d ' ' | tr '\n' ' ')"
    fi
else
    warn "speed_limits" "不存在：将使用 global_planning.yaml 的占位上限（1000 mm/s 偏快），现场 §7 生成"
fi

ODIN_CFG="$WORKSPACE/src/odin_ros_driver/config/control_command.yaml"
if [ -f "$ODIN_CFG" ]; then
    # custom_map_mode 藏在 register_keys: 下面，递归找（找不到再退回文本匹配）
    MODE="$(python3 - "$ODIN_CFG" <<'PY' 2>/dev/null
import re, sys, yaml

path = sys.argv[1]
def walk(node):
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "custom_map_mode":
                return value
            found = walk(value)
            if found is not None:
                return found
    return None

found = walk(yaml.safe_load(open(path)) or {})
if found is None:
    match = re.search(r"^\s*custom_map_mode:\s*(\S+)", open(path).read(), re.M)
    found = match.group(1) if match else None
print("" if found is None else found)
PY
)"
    if [ "$MODE" = "0" ]; then
        ok "odin-mode" "custom_map_mode=0（纯里程计，本方案要求）"
    elif [ -z "$MODE" ]; then
        warn "odin-mode" "配置里读不到 custom_map_mode —— 手工确认应为 0（纯里程计）"
    else
        bad "odin-mode" "custom_map_mode=$MODE != 0 —— 本方案要求 0；改配置或重新决策定位方案"
    fi
fi

# ---------------------------------------------------------------- 5. 设备
if [ "$CHECK_DEVICES" = "1" ]; then
    sec "5. 设备与权限"

    UDEV_RULE=/etc/udev/rules.d/99-odin-usb.rules
    if [ "$CHECK_ODIN" = "0" ]; then
        warn "udev-odin" "已按 --no-odin 跳过（台架/开发机）"
    elif [ -f "$UDEV_RULE" ] && grep -q "2207" "$UDEV_RULE"; then
        ok "udev-odin" "$UDEV_RULE 已配置（idVendor 2207 / idProduct 0019）"
    else
        bad "udev-odin" "缺 $UDEV_RULE —— odin1 会没权限/认不到；按 odin README §3.1 建规则后 udevadm control --reload && udevadm trigger"
    fi

    if command -v lsusb > /dev/null 2>&1; then
        if lsusb -d 2207:0019 > /dev/null 2>&1; then
            ok "odin-usb" "$(lsusb -d 2207:0019 | head -1 | cut -c1-70)"
        else
            bad "odin-usb" "lsusb 看不到 2207:0019 —— 线/供电/设备问题；里程计无数据则整车全自动不可用"
        fi
    else
        warn "odin-usb" "无 lsusb（装 usbutils 才能查）"
    fi

    if [ -f /usr/local/lib/libusb-1.0.so ]; then
        ok "libusb-preload" "/usr/local/lib/libusb-1.0.so 存在（chassis.sh 的 LD_PRELOAD 生效）"
    else
        warn "libusb-preload" "缺 /usr/local/lib/libusb-1.0.so —— 开发机正常；**车上缺失会让 odin 连接失败**，需先装匹配版本"
    fi

    if [ -z "$SERIAL" ]; then
        SERIAL="$(python3 -c "import yaml;print((yaml.safe_load(open('$WORKSPACE/src/uartdriver/config/driver_config.yaml')) or {}).get('/uart_driver_node',{}).get('ros__parameters',{}).get('serial_name','/dev/ttyACM0'))" 2>/dev/null)"
        [ -z "$SERIAL" ] && SERIAL=/dev/ttyACM0
    fi
    if [ -e "$SERIAL" ]; then
        if [ -r "$SERIAL" ] && [ -w "$SERIAL" ]; then
            ok "serial" "$SERIAL 可读写（$(basename "$(readlink -f "$SERIAL")")）"
        else
            if sudo -n true 2>/dev/null; then
                ok "serial" "$SERIAL 存在但当前用户不可读写 —— scripts/chassis.sh 会 sudo chmod 777（免密 sudo 可用）"
            else
                bad "serial" "$SERIAL 权限不足且 sudo 需要密码 —— 先 sudo chmod 666 $SERIAL 或把用户加进 dialout"
            fi
        fi
    else
        bad "serial" "$SERIAL 不存在 —— 查 USB 线/电控是否上电；ls /dev/ttyUSB* /dev/ttyACM*"
    fi
fi

# ---------------------------------------------------------------- 6. DDS
sec "6. DDS / 网络"
if [ -n "${ROS_DOMAIN_ID:-}" ]; then
    ok "ros-domain" "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
else
    ok "ros-domain" "ROS_DOMAIN_ID 未设置（默认 0，单机比赛正常）"
fi
if [ -n "${FASTDDS_BUILTIN_TRANSPORTS:-}" ]; then
    ok "dds-transport" "FASTDDS_BUILTIN_TRANSPORTS=$FASTDDS_BUILTIN_TRANSPORTS（同机联调推荐 SHM）"
else
    warn "dds-transport" "未设置：本机/台架/WSL 上跨进程发现可能 0 帧，联调时两个进程都 export FASTDDS_BUILTIN_TRANSPORTS=SHM"
fi

# ---------------------------------------------------------------- 总结
echo
echo "--------------------------------------------------------------"
printf ' 总结: \033[32mPASS %d\033[0m  \033[33mWARN %d\033[0m  \033[31mFAIL %d\033[0m\n' "$PASS" "$WARN" "$FAIL"
if [ "$FAIL" -gt 0 ]; then
    echo " 结论: 有 FAIL —— 先修掉再上车；FAIL 项就是后面会直接导致跑不起来的项"
    exit 1
fi
if [ "$STRICT" = "1" ] && [ "$WARN" -gt 0 ]; then
    echo " 结论: --strict 下有 WARN —— 出发前在驻地要求全绿"
    exit 1
fi
echo " 结论: 可用（WARN 项按需处理）"
exit 0
