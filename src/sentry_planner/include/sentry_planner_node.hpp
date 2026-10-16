#pragma once

#include "common.h"
#include "nav_map.hpp"
#include "waypoint_graph.hpp"

class SentryPlannerNode : public rclcpp::Node {
public:
    SentryPlannerNode(std::string name);

private:
    rclcpp::Subscription<OdomMsg>::SharedPtr odom_suber;
    rclcpp::Subscription<ChassisDataMsg>::SharedPtr chassis_data_suber;
    rclcpp::Subscription<NavCommandMsg>::SharedPtr nav_command_suber;

    rclcpp::Publisher<VelocityCommandMsg>::SharedPtr velocity_puber;
    rclcpp::TimerBase::SharedPtr control_timer;

    // 坐标变换接收者
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // 参数（m 为长度单位，速度指令为 mm/s、deg/s）
    std::string global_frame;
    std::string robot_frame;
    std::string nav_map_file;             // 空 = 自动用 <share>/map/701_model/nav_map.bin
    std::string waypoint_graph_file;      // 空 = 自动用 <share>/config/waypoint_graph.yaml
    bool require_map = true;              // true：地图/路径图缺失则拒绝运动（真车默认）
    bool require_chassis_data = true;     // true：底盘(串口)数据中断则停车（真车默认）
    double control_rate_hz = 50.0;
    double command_timeout = 1.0;         // s：决策指令超时则停车
    double arrive_radius = 0.15;          // m：默认到位半径（NavCommand.radius 为 NaN 时用）
    double hold_radius = 0.05;            // m：MODE_HOLD 的保持死区
    double hold_gain = 0.8;               // 1/s：驻留修正增益（比 pos_kp 小，避免抖动）
    double max_hold_speed = 200.0;        // mm/s：驻留修正的速度上限
    double pos_kp = 1.5;                  // 1/s：位置 P 增益（输出 mm/s）
    double yaw_kp = 2.0;                  // 1/s：朝向 P 增益（输出 deg/s）
    double max_vx = 1000.0;               // mm/s
    double max_vy = 1000.0;               // mm/s
    double max_wz = 360.0;                // deg/s
    // 加速度限幅：电控不做功率控制（QA2），100 W 上限完全靠命令变化率兜住
    double max_acc_vx = 2000.0;           // mm/s^2
    double max_acc_vy = 2000.0;           // mm/s^2
    double max_acc_wz = 720.0;            // deg/s^2
    double lookahead_dist = 0.60;         // m：纯跟踪前视距离
    double path_margin = 0.35;            // m：规划安全边距（车体半径 + 余量）
    double stuck_cmd_speed = 100.0;       // mm/s：指令速度高于该值才做卡住检测
    double stuck_chassis_speed = 20.0;    // mm/s：底盘反馈速度低于该值视为没在动
    double stuck_timeout = 2.0;           // s
    double odom_timeout = 0.5;            // s：里程计中断则停车
    double chassis_timeout = 0.5;         // s：底盘数据中断则停车

    // 地图与路径图
    NavMap nav_map;
    WaypointGraph graph;

    // 状态
    bool command_received = false;
    NavCommandMsg nav_command;
    rclcpp::Time command_stamp;

    bool odom_received = false;
    rclcpp::Time odom_stamp;

    bool chassis_data_received = false;
    ChassisDataMsg chassis_data;
    rclcpp::Time chassis_stamp;

    bool stuck_detected = false;
    rclcpp::Time stuck_start;

    // 上一次发布的指令速度（用于加速度限幅）
    double last_vx = 0.0, last_vy = 0.0, last_wz = 0.0;
    rclcpp::Time last_velocity_stamp;

    // 路径状态（当前目标对应的折线，世界系）
    std::vector<Point2D> path;
    bool planned_once = false;
    double planned_x = 0.0;
    double planned_y = 0.0;
    int planned_mode = -1;

    void ChassisInfoCallback(const ChassisDataMsg::SharedPtr msg);
    void OdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
    void NavCommandCallback(const NavCommandMsg::SharedPtr msg);

    void ControlLoop();
    void publish_velocity(double vx, double vy, double wz);
    void stop_velocity();
    void check_stuck(double vx, double vy);
    static double slew_towards(double current, double target, double max_delta);

    // 地图/路径图加载与自检
    void loadMapAndGraph();
    // 组装「车 -> 路径图节点序列 -> 目标」的折线；无解返回 false（此时不允许运动）
    bool buildPath(double x0, double y0, double gx, double gy);
    // 纯跟踪：返回前视点（世界系）
    Point2D lookaheadPoint(double x, double y) const;
    // 本帧生效的到位半径
    double targetRadius() const;
    double pathLength() const;
};
