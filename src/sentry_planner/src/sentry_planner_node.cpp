#include "sentry_planner_node.hpp"

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

    // 控制参数
    declare_parameter("control_rate_hz", 50.0);
    control_rate_hz = get_parameter("control_rate_hz").as_double();

    declare_parameter("command_timeout", 1.0);
    command_timeout = get_parameter("command_timeout").as_double();

    declare_parameter("arrive_radius", 0.15);
    arrive_radius = get_parameter("arrive_radius").as_double();

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

    declare_parameter("stuck_cmd_speed", 100.0);
    stuck_cmd_speed = get_parameter("stuck_cmd_speed").as_double();

    declare_parameter("stuck_chassis_speed", 20.0);
    stuck_chassis_speed = get_parameter("stuck_chassis_speed").as_double();

    declare_parameter("stuck_timeout", 2.0);
    stuck_timeout = get_parameter("stuck_timeout").as_double();

    declare_parameter("odom_timeout", 0.5);
    odom_timeout = get_parameter("odom_timeout").as_double();

    log_parameters(this, {"odom_topic", "chassis_data_topic", "nav_command_topic",
                          "velocity_command_topic", "global_frame", "robot_frame",
                          "control_rate_hz", "command_timeout", "arrive_radius",
                          "pos_kp", "yaw_kp", "max_vx", "max_vy", "max_wz",
                          "stuck_cmd_speed", "stuck_chassis_speed", "stuck_timeout",
                          "odom_timeout"});

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

    control_timer = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_hz),
        std::bind(&SentryPlannerNode::ControlLoop, this));

    info_log(name, "has been built!");
}

// 底盘信息回调：用于卡住检测
void SentryPlannerNode::ChassisInfoCallback(const ChassisDataMsg::SharedPtr msg) {
    chassis_data = *msg;
    chassis_data_received = true;
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

void SentryPlannerNode::ControlLoop() {
    // 1. 指令有效性：未收到指令 / 急停 / 指令超时 -> 零速
    if(!command_received || nav_command.mode == NavCommandMsg::MODE_STOP){
        publish_velocity(0.0, 0.0, 0.0);
        return;
    }

    const double command_age = (now() - command_stamp).seconds();
    if(command_age > command_timeout){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "nav command timeout (%.3f s > %.3f s), stop", command_age, command_timeout);
        publish_velocity(0.0, 0.0, 0.0);
        return;
    }

    // 里程计中断 -> 停车（此时 tf 也不可靠）
    if(!odom_received || (now() - odom_stamp).seconds() > odom_timeout){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "odometry timeout (>%.3f s), stop", odom_timeout);
        publish_velocity(0.0, 0.0, 0.0);
        return;
    }

    // 2. 位姿：global_frame -> robot_frame
    geometry_msgs::msg::TransformStamped tf_stamped;
    try{
        tf_stamped = tf_buffer->lookupTransform(global_frame, robot_frame, tf2::TimePointZero);
    }catch(const tf2::TransformException &ex){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "tf %s -> %s unavailable: %s", global_frame.c_str(), robot_frame.c_str(), ex.what());
        publish_velocity(0.0, 0.0, 0.0);
        return;
    }

    const double yaw = tf2::getYaw(tf_stamped.transform.rotation);
    const double err_x = nav_command.x - tf_stamped.transform.translation.x;
    const double err_y = nav_command.y - tf_stamped.transform.translation.y;
    const double distance = std::hypot(err_x, err_y);

    // 3. 位置 P 控制（世界系 mm/s）-> 车体系
    double world_vx = 0.0, world_vy = 0.0;
    if(distance > arrive_radius){
        world_vx = pos_kp * err_x * 1000.0;
        world_vy = pos_kp * err_y * 1000.0;
    }
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    double vx = std::clamp(cos_yaw * world_vx + sin_yaw * world_vy, -max_vx, max_vx);
    double vy = std::clamp(-sin_yaw * world_vx + cos_yaw * world_vy, -max_vy, max_vy);

    // 4. 旋转：小陀螺优先；否则朝目标朝向闭环（wz 为唯一旋转通道，二者互斥）
    double wz = 0.0;
    if(nav_command.spin_rate != 0.0f){
        wz = nav_command.spin_rate;
    }else if(!std::isnan(nav_command.yaw)){
        const double yaw_err = std::remainder(nav_command.yaw - yaw, 2.0 * std::numbers::pi);
        wz = yaw_kp * yaw_err * 180.0 / std::numbers::pi;
    }
    wz = std::clamp(wz, -max_wz, max_wz);

    check_stuck(vx, vy);
    publish_velocity(vx, vy, wz);
}
