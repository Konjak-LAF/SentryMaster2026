#include "common.h"

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
    double control_rate_hz = 50.0;
    double command_timeout = 1.0;      // s：决策指令超时则停车
    double arrive_radius = 0.15;       // m：小于该距离不再平移
    double pos_kp = 1.5;               // 1/s：位置 P 增益（输出 mm/s）
    double yaw_kp = 2.0;               // 1/s：朝向 P 增益（输出 deg/s）
    double max_vx = 1000.0;            // mm/s
    double max_vy = 1000.0;            // mm/s
    double max_wz = 360.0;             // deg/s
    double stuck_cmd_speed = 100.0;    // mm/s：指令速度高于该值才做卡住检测
    double stuck_chassis_speed = 20.0; // mm/s：底盘反馈速度低于该值视为没在动
    double stuck_timeout = 2.0;        // s
    double odom_timeout = 0.5;         // s：里程计中断则停车

    // 状态
    bool command_received = false;
    NavCommandMsg nav_command;
    rclcpp::Time command_stamp;

    bool odom_received = false;
    rclcpp::Time odom_stamp;

    bool chassis_data_received = false;
    ChassisDataMsg chassis_data;

    bool stuck_detected = false;
    rclcpp::Time stuck_start;

    void ChassisInfoCallback(const ChassisDataMsg::SharedPtr msg);
    void OdomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
    void NavCommandCallback(const NavCommandMsg::SharedPtr msg);

    void ControlLoop();
    void publish_velocity(double vx, double vy, double wz);
    void check_stuck(double vx, double vy);
};
