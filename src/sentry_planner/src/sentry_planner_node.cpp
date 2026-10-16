#include "sentry_planner_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

SentryPlannerNode::SentryPlannerNode(std::string name):Node(name){
    tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()
    );
    tf_buffer->setCreateTimerInterface(timer_interface);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // 话题参数
    declare_parameter("odom_topic", "/odin1/odometry_highfreq");
    const auto odom_topic = get_parameter("odom_topic").as_string();

    declare_parameter("chassis_data_topic", "/chassis_data");
    const auto chassis_topic = get_parameter("chassis_data_topic").as_string();

    declare_parameter("nav_command_topic", "/sentry/nav_command");
    const auto nav_command_topic = get_parameter("nav_command_topic").as_string();

    declare_parameter("velocity_command_topic", "/sentry/velocity_command");
    const auto velocity_topic = get_parameter("velocity_command_topic").as_string();

    // 坐标系参数
    declare_parameter("global_frame", "odom");
    global_frame = get_parameter("global_frame").as_string();

    declare_parameter("robot_frame", "vehicle");
    robot_frame = get_parameter("robot_frame").as_string();

    // 地图与路径图
    declare_parameter("nav_map_file", "");
    nav_map_file = get_parameter("nav_map_file").as_string();

    declare_parameter("waypoint_graph_file", "");
    waypoint_graph_file = get_parameter("waypoint_graph_file").as_string();

    declare_parameter("require_map", true);
    require_map = get_parameter("require_map").as_bool();

    // 控制参数
    declare_parameter("control_rate_hz", 50.0);
    control_rate_hz = get_parameter("control_rate_hz").as_double();

    declare_parameter("command_timeout", 1.0);
    command_timeout = get_parameter("command_timeout").as_double();

    declare_parameter("arrive_radius", 0.15);
    arrive_radius = get_parameter("arrive_radius").as_double();

    declare_parameter("hold_radius", 0.05);
    hold_radius = get_parameter("hold_radius").as_double();

    declare_parameter("hold_gain", 0.8);
    hold_gain = get_parameter("hold_gain").as_double();

    declare_parameter("max_hold_speed", 200.0);
    max_hold_speed = get_parameter("max_hold_speed").as_double();

    declare_parameter("pos_kp", 1.5);
    pos_kp = get_parameter("pos_kp").as_double();

    declare_parameter("yaw_kp", 2.0);
    yaw_kp = get_parameter("yaw_kp").as_double();

    declare_parameter("max_vx", 1000.0);
    max_vx = get_parameter("max_vx").as_double();

    declare_parameter("max_vy", 1000.0);
    max_vy = get_parameter("max_vy").as_double();

    declare_parameter("max_wz", 360.0);
    max_wz = get_parameter("max_wz").as_double();

    declare_parameter("max_acc_vx", 2000.0);
    max_acc_vx = get_parameter("max_acc_vx").as_double();

    declare_parameter("max_acc_vy", 2000.0);
    max_acc_vy = get_parameter("max_acc_vy").as_double();

    declare_parameter("max_acc_wz", 720.0);
    max_acc_wz = get_parameter("max_acc_wz").as_double();

    declare_parameter("lookahead_dist", 0.60);
    lookahead_dist = get_parameter("lookahead_dist").as_double();

    declare_parameter("path_margin", 0.35);
    path_margin = get_parameter("path_margin").as_double();

    declare_parameter("stuck_cmd_speed", 100.0);
    stuck_cmd_speed = get_parameter("stuck_cmd_speed").as_double();

    declare_parameter("stuck_chassis_speed", 20.0);
    stuck_chassis_speed = get_parameter("stuck_chassis_speed").as_double();

    declare_parameter("stuck_timeout", 2.0);
    stuck_timeout = get_parameter("stuck_timeout").as_double();

    declare_parameter("odom_timeout", 0.5);
    odom_timeout = get_parameter("odom_timeout").as_double();

    declare_parameter("chassis_timeout", 0.5);
    chassis_timeout = get_parameter("chassis_timeout").as_double();

    declare_parameter("require_chassis_data", true);
    require_chassis_data = get_parameter("require_chassis_data").as_bool();

    log_parameters(this, {"odom_topic", "chassis_data_topic", "nav_command_topic",
                          "velocity_command_topic", "global_frame", "robot_frame",
                          "nav_map_file", "waypoint_graph_file", "require_map",
                          "require_chassis_data", "control_rate_hz", "command_timeout",
                          "arrive_radius", "hold_radius", "hold_gain", "max_hold_speed",
                          "pos_kp", "yaw_kp", "max_vx", "max_vy", "max_wz",
                          "max_acc_vx", "max_acc_vy", "max_acc_wz",
                          "lookahead_dist", "path_margin",
                          "stuck_cmd_speed", "stuck_chassis_speed", "stuck_timeout",
                          "odom_timeout", "chassis_timeout"});

    if(control_rate_hz <= 0.0){
        error_log("control_rate_hz", control_rate_hz, "is invalid, use 50.0 instead");
        control_rate_hz = 50.0;
    }

    odom_suber = create_subscription<OdomMsg>(
        odom_topic, 10, std::bind(&SentryPlannerNode::OdomCallBack, this, std::placeholders::_1));
    chassis_data_suber = create_subscription<ChassisDataMsg>(
        chassis_topic, 10, std::bind(&SentryPlannerNode::ChassisInfoCallback, this, std::placeholders::_1));
    nav_command_suber = create_subscription<NavCommandMsg>(
        nav_command_topic, 10, std::bind(&SentryPlannerNode::NavCommandCallback, this, std::placeholders::_1));

    velocity_puber = create_publisher<VelocityCommandMsg>(velocity_topic, 10);

    loadMapAndGraph();

    control_timer = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_hz),
        std::bind(&SentryPlannerNode::ControlLoop, this));

    info_log(name, "has been built!");
}

void SentryPlannerNode::loadMapAndGraph() {
    std::string share_dir;
    try{
        share_dir = ament_index_cpp::get_package_share_directory("sentry_planner");
    }catch(const std::exception &ex){
        error_log("[map] cannot resolve sentry_planner share dir:", ex.what());
    }

    const std::string map_path = nav_map_file.empty()
        ? share_dir + "/map/701_model/nav_map.bin" : nav_map_file;
    const std::string graph_path = waypoint_graph_file.empty()
        ? share_dir + "/config/waypoint_graph.yaml" : waypoint_graph_file;

    if(nav_map.load(map_path)){
        info_log("[map] loaded", map_path, "grid", nav_map.width(), "x", nav_map.height(),
                 "@", nav_map.resolution(), "m  source:", nav_map.source());
    }else{
        error_log("[map] load failed:", nav_map.status());
    }

    if(graph.load(graph_path)){
        std::string detail;
        const int blocked = graph.blockedEdges(nav_map, path_margin, &detail);
        info_log("[graph] loaded", graph_path, "nodes", graph.nodes().size(),
                 "edges", graph.edges().size(), "禁转区", graph.spinZones().size(),
                 "不可通行边", blocked, blocked > 0 ? ("(" + detail + ")") : "");
        if(blocked > 0){
            RCLCPP_WARN(get_logger(), "[graph] %d 条边在当前地图 + path_margin=%.2fm 下不可通行，"
                                      "规划时会跳过；请核对打点或 path_margin",
                        blocked, path_margin);
        }
    }else{
        error_log("[graph] load failed:", graph.status());
    }

    if(require_map && (!nav_map.ready() || !graph.ready())){
        RCLCPP_ERROR(get_logger(),
                     "[fault] 地图/路径图不可用（map=%s graph=%s），require_map=true -> 拒绝运动。"
                     "离线生成地图见 tools/build_nav_map.py，现场打点见 tools/mark_points.py（4.md）",
                     nav_map.ready() ? "ok" : nav_map.status(),
                     graph.ready() ? "ok" : graph.status());
    }else if(!nav_map.ready() || !graph.ready()){
        RCLCPP_WARN(get_logger(), "[map] 地图/路径图不可用，退化为直线跟踪（仅限台架/仿真）");
    }
}

// 底盘信息回调：用于卡住检测 + 串口看门狗
void SentryPlannerNode::ChassisInfoCallback(const ChassisDataMsg::SharedPtr msg) {
    chassis_data = *msg;
    chassis_data_received = true;
    chassis_stamp = now();
}

// 定位节点位置回调：位姿统一走 tf，这里只用于判断里程计是否中断
void SentryPlannerNode::OdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
    (void)msg;
    odom_received = true;
    odom_stamp = now();
}

// 决策指令回调
void SentryPlannerNode::NavCommandCallback(const NavCommandMsg::SharedPtr msg) {
    nav_command = *msg;
    command_stamp = now();
    command_received = true;
}

void SentryPlannerNode::publish_velocity(double vx, double vy, double wz) {
    VelocityCommandMsg msg;
    msg.header.stamp = now();
    msg.header.frame_id = robot_frame;
    msg.vx = static_cast<float>(vx);
    msg.vy = static_cast<float>(vy);
    msg.wz = static_cast<float>(wz);
    velocity_puber->publish(msg);
}

// 停车：安全优先，不做加速度限幅（立即归零），同时把限幅状态清零
void SentryPlannerNode::stop_velocity() {
    last_vx = 0.0;
    last_vy = 0.0;
    last_wz = 0.0;
    last_velocity_stamp = now();
    planned_once = false;             // 再启动时重新规划（停车期间车可能被挪动）
    publish_velocity(0.0, 0.0, 0.0);
}

// 把 current 朝 target 移动，单步变化不超过 max_delta
double SentryPlannerNode::slew_towards(double current, double target, double max_delta) {
    const double delta = target - current;
    if(std::abs(delta) <= max_delta){
        return target;
    }
    return current + std::copysign(max_delta, delta);
}

// 卡住检测：指令速度不为零，但底盘反馈长时间几乎不动
void SentryPlannerNode::check_stuck(double vx, double vy) {
    if(!chassis_data_received || std::hypot(vx, vy) < stuck_cmd_speed){
        stuck_detected = false;
        return;
    }

    const double chassis_speed = std::hypot(chassis_data.vx, chassis_data.vy);
    if(chassis_speed > stuck_chassis_speed){
        stuck_detected = false;
        return;
    }

    const rclcpp::Time stamp = now();
    if(!stuck_detected){
        stuck_detected = true;
        stuck_start = stamp;
        return;
    }

    const double duration = (stamp - stuck_start).seconds();
    if(duration > stuck_timeout){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "robot may be stuck: command %.1f mm/s but chassis feedback %.1f mm/s for %.1f s",
            std::hypot(vx, vy), chassis_speed, duration);
    }
}

double SentryPlannerNode::targetRadius() const {
    if(nav_command.mode == NavCommandMsg::MODE_HOLD){
        return hold_radius;
    }
    if(std::isnan(nav_command.radius) || nav_command.radius <= 0.0f){
        return arrive_radius;
    }
    return static_cast<double>(nav_command.radius);
}

double SentryPlannerNode::pathLength() const {
    double total = 0.0;
    for(std::size_t i = 0; i + 1 < path.size(); ++i){
        total += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    }
    return total;
}

Point2D SentryPlannerNode::lookaheadPoint(double x, double y) const {
    if(path.size() < 2){
        return Point2D{static_cast<double>(nav_command.x), static_cast<double>(nav_command.y)};
    }

    // 1. 找到路径上离车最近的投影点
    std::size_t best_segment = 0;
    double best_ratio = 0.0;
    double best_distance = std::numeric_limits<double>::infinity();
    for(std::size_t i = 0; i + 1 < path.size(); ++i){
        const double dx = path[i + 1].x - path[i].x;
        const double dy = path[i + 1].y - path[i].y;
        const double length_sq = dx * dx + dy * dy;
        double ratio = 0.0;
        if(length_sq > 1e-12){
            ratio = ((x - path[i].x) * dx + (y - path[i].y) * dy) / length_sq;
            ratio = std::clamp(ratio, 0.0, 1.0);
        }
        const double px = path[i].x + dx * ratio;
        const double py = path[i].y + dy * ratio;
        const double distance = std::hypot(px - x, py - y);
        if(distance < best_distance){
            best_distance = distance;
            best_segment = i;
            best_ratio = ratio;
        }
    }

    // 2. 从投影点沿路径前进 lookahead_dist
    double travelled = 0.0;
    Point2D cursor{
        path[best_segment].x + (path[best_segment + 1].x - path[best_segment].x) * best_ratio,
        path[best_segment].y + (path[best_segment + 1].y - path[best_segment].y) * best_ratio};
    for(std::size_t i = best_segment; i + 1 < path.size(); ++i){
        const Point2D start = (i == best_segment) ? cursor : path[i];
        const Point2D end = path[i + 1];
        const double segment = std::hypot(end.x - start.x, end.y - start.y);
        if(travelled + segment >= lookahead_dist){
            const double need = lookahead_dist - travelled;
            const double ratio = (segment > 1e-9) ? need / segment : 0.0;
            return Point2D{start.x + (end.x - start.x) * ratio, start.y + (end.y - start.y) * ratio};
        }
        travelled += segment;
    }
    return path.back();
}

bool SentryPlannerNode::buildPath(double x0, double y0, double gx, double gy) {
    path.clear();
    path.push_back(Point2D{x0, y0});
    path.push_back(Point2D{gx, gy});

    if(!nav_map.ready() || !graph.ready()){
        return true;                       // 退化：直线（台架/仿真，require_map=false 时）
    }

    const Point2D start{x0, y0};
    const Point2D goal{gx, gy};

    // 终点接入点：离目标最近、且「节点 -> 目标」直线可通行的节点
    const int entry = graph.nearestReachable(goal, nav_map, path_margin);
    if(entry < 0){
        const bool direct_ok = nav_map.lineClear(start.x, start.y, goal.x, goal.y, path_margin);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[plan] 目标 (%.2f, %.2f) 附近没有可通行接入点；%s",
            goal.x, goal.y, direct_ok ? "改用直线（已确认可通行）" : "无可行路径，停车");
        return direct_ok;
    }

    const int start_node = graph.nearest(start);
    if(start_node < 0){
        return nav_map.lineClear(start.x, start.y, goal.x, goal.y, path_margin);
    }
    if(!nav_map.lineClear(start.x, start.y, graph.nodes()[start_node].p.x,
                          graph.nodes()[start_node].p.y, path_margin)){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[plan] 车体当前位置距最近节点 %s 的连线不满足 %.2fm 安全边距（可能贴墙/漂移），"
            "仍按该节点出发",
            graph.nodes()[start_node].name.c_str(), path_margin);
    }

    std::vector<int> sequence;
    if(!graph.plan(start_node, entry, nav_map, path_margin, sequence)){
        const bool direct_ok = nav_map.lineClear(start.x, start.y, goal.x, goal.y, path_margin);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[plan] 路径图无解（%s -> %s）；%s",
            graph.nodes()[start_node].name.c_str(), graph.nodes()[entry].name.c_str(),
            direct_ok ? "改用直线（已确认可通行）" : "无可行路径，停车");
        return direct_ok;
    }

    path.clear();
    path.push_back(start);
    for(const int index : sequence){
        path.push_back(graph.nodes()[index].p);
    }
    path.push_back(goal);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[plan] %zu 个点 %.2fm：%s%s%s", path.size(), pathLength(),
        graph.nodes()[start_node].name.c_str(), (entry == start_node) ? "" : " ... ",
        (entry == start_node) ? "" : graph.nodes()[entry].name.c_str());
    return true;
}

void SentryPlannerNode::ControlLoop() {
    // 1. 指令有效性：未收到指令 / 急停 / 指令超时 -> 零速
    if(!command_received || nav_command.mode == NavCommandMsg::MODE_STOP){
        stop_velocity();
        return;
    }

    const double command_age = (now() - command_stamp).seconds();
    if(command_age > command_timeout){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "nav command timeout (%.3f s > %.3f s), stop", command_age, command_timeout);
        stop_velocity();
        return;
    }

    // 里程计中断 -> 停车（此时 tf 也不可靠）
    if(!odom_received || (now() - odom_stamp).seconds() > odom_timeout){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "odometry timeout (>%.3f s), stop", odom_timeout);
        stop_velocity();
        return;
    }

    // 底盘/串口数据中断 -> 停车（决策侧同样按 referee/chassis 超时降级，见 2.md P3.7）
    if(require_chassis_data &&
       (!chassis_data_received || (now() - chassis_stamp).seconds() > chassis_timeout)){
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
            "[fault] chassis/serial data timeout (>%.3f s), stop", chassis_timeout);
        stop_velocity();
        return;
    }

    if(require_map && (!nav_map.ready() || !graph.ready())){
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
            "[fault] map/graph unavailable (require_map=true), stop");
        stop_velocity();
        return;
    }

    // 2. 位姿：global_frame -> robot_frame
    geometry_msgs::msg::TransformStamped tf_stamped;
    try{
        tf_stamped = tf_buffer->lookupTransform(global_frame, robot_frame, tf2::TimePointZero);
    }catch(const tf2::TransformException &ex){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "tf %s -> %s unavailable: %s", global_frame.c_str(), robot_frame.c_str(), ex.what());
        stop_velocity();
        return;
    }

    const double yaw = tf2::getYaw(tf_stamped.transform.rotation);
    const double x = tf_stamped.transform.translation.x;
    const double y = tf_stamped.transform.translation.y;
    const double target_x = static_cast<double>(nav_command.x);
    const double target_y = static_cast<double>(nav_command.y);
    const double distance = std::hypot(target_x - x, target_y - y);
    const double radius = targetRadius();

    // 3. 目标变化 -> 重新规划
    const bool target_changed = !planned_once ||
        std::hypot(target_x - planned_x, target_y - planned_y) > 0.02 ||
        nav_command.mode != planned_mode;
    if(target_changed){
        if(!buildPath(x, y, target_x, target_y)){
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "[plan] no path to (%.2f, %.2f), stop", target_x, target_y);
            stop_velocity();
            return;
        }
        planned_x = target_x;
        planned_y = target_y;
        planned_mode = nav_command.mode;
        planned_once = true;
    }

    // 4. 平移：GOTO 走纯跟踪；HOLD 做小增益位置保持
    double world_vx = 0.0;
    double world_vy = 0.0;
    if(nav_command.mode == NavCommandMsg::MODE_HOLD){
        if(distance > radius){
            world_vx = hold_gain * (target_x - x) * 1000.0;
            world_vy = hold_gain * (target_y - y) * 1000.0;
            const double speed = std::hypot(world_vx, world_vy);
            if(speed > max_hold_speed){
                world_vx *= max_hold_speed / speed;
                world_vy *= max_hold_speed / speed;
            }
        }
    }else if(distance > radius){
        const Point2D lookahead = lookaheadPoint(x, y);
        world_vx = pos_kp * (lookahead.x - x) * 1000.0;
        world_vy = pos_kp * (lookahead.y - y) * 1000.0;
        // 末段减速：v <= sqrt(2*a*s)，避免高速冲过目标点（a 取平移加速度限幅的较小者）
        // 注意单位：distance 是 m，decel/speed 是 mm，s 要 ×1000
        const double decel = std::min(max_acc_vx, max_acc_vy);
        const double slowing_distance = std::max(0.0, distance - radius) * 1000.0;   // mm
        const double speed_limit = std::sqrt(2.0 * decel * slowing_distance);        // mm/s
        const double speed = std::hypot(world_vx, world_vy);
        if(speed > speed_limit && speed > 1e-9){
            const double scale = speed_limit / speed;
            world_vx *= scale;
            world_vy *= scale;
        }
    }

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    double vx = std::clamp(cos_yaw * world_vx + sin_yaw * world_vy, -max_vx, max_vx);
    double vy = std::clamp(-sin_yaw * world_vx + cos_yaw * world_vy, -max_vy, max_vy);

    // 5. 旋转：小陀螺优先（禁转区内强制关掉）；否则朝目标朝向闭环（wz 为唯一旋转通道，二者互斥）
    double wz = 0.0;
    if(nav_command.spin_rate != 0.0f){
        const SpinForbiddenZone *zone = graph.spinZoneAt(x, y);
        if(zone != nullptr){
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[spin] 当前位于禁小陀螺区 %s，忽略决策的 %.0f deg/s（避免斜坡上超功率）",
                zone->name.c_str(), nav_command.spin_rate);
        }else{
            wz = nav_command.spin_rate;
        }
    }else if(!std::isnan(nav_command.yaw)){
        const double yaw_err = std::remainder(static_cast<double>(nav_command.yaw) - yaw,
                                              2.0 * std::numbers::pi);
        wz = yaw_kp * yaw_err * 180.0 / std::numbers::pi;
    }
    wz = std::clamp(wz, -max_wz, max_wz);

    // 6. 加速度限幅：电控不做功率控制，功率上限只能靠命令变化率兜住
    //    （停车/急停不受限：安全优先，直接归零）
    const rclcpp::Time stamp = now();
    double dt = 1.0 / control_rate_hz;   // 首拍按一个控制周期限幅（否则第一次下发会直接跳满）
    if(last_velocity_stamp.nanoseconds() > 0){
        dt = (stamp - last_velocity_stamp).seconds();
        dt = std::clamp(dt, 0.0, 0.2);
    }
    if(dt > 0.0){
        vx = slew_towards(last_vx, vx, max_acc_vx * dt);
        vy = slew_towards(last_vy, vy, max_acc_vy * dt);
        wz = slew_towards(last_wz, wz, max_acc_wz * dt);
    }
    last_vx = vx;
    last_vy = vy;
    last_wz = wz;
    last_velocity_stamp = stamp;

    check_stuck(vx, vy);
    publish_velocity(vx, vy, wz);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[nav] mode=%u tgt=(%.2f, %.2f) pose=(%.2f, %.2f, %.0fdeg) d=%.2f r=%.2f path=%.2fm "
        "cmd=(%.0f, %.0f, %.0f)",
        nav_command.mode, target_x, target_y, x, y, yaw * 180.0 / std::numbers::pi,
        distance, radius, pathLength(), vx, vy, wz);
}
