#pragma once
#include "common.h"
#include "serial/serial.h"
#include "struct_def.hpp"
#include <mutex>

// 发送链路参数（由节点从 yaml 读出后传入）
struct TransferConfig{
    double velocity_timeout = 0.2;   // s：超过该时长没有新的速度指令 -> 发零速
    double posture_timeout = 1.0;    // s：姿态指令超时 -> 回到 default_posture
    uint8_t default_posture = 3;     // 未收到姿态指令（或超时）时使用的姿态（3 移动姿态）
};

class Transfer{
private:
    rclcpp::Node *node = nullptr;

    // 引用节点持有的串口句柄：重连会替换对象，引用保证发送端总是拿到最新句柄
    std::shared_ptr<serial::Serial> &serial_port;

    rclcpp::Publisher<RefereeMsg>::SharedPtr referee_puber;
    rclcpp::Publisher<ChassisMsg>::SharedPtr chassis_puber;
    rclcpp::Subscription<VelocityMsg>::SharedPtr velocity_suber;
    rclcpp::Subscription<PostureMsg>::SharedPtr posture_suber;

    TransferConfig config;

    std::mutex cmd_mutex;              // 保护下面 6 个指令缓存
    VelocityMsg velocity_cmd;
    uint8_t posture_cmd = 3;
    bool velocity_received = false;
    bool posture_received = false;
    rclcpp::Time velocity_stamp;
    rclcpp::Time posture_stamp;

    std::mutex write_mutex;            // 串口写互斥（发送侧唯一写者）

    bool send_serial_packet(const std::vector<uint8_t> &datas, const std::string &tag);
    void velocity_callback(const VelocityMsg::SharedPtr msg);
    void posture_callback(const PostureMsg::SharedPtr msg);

public:
    Transfer(rclcpp::Node *node, std::shared_ptr<serial::Serial> &_serial_port,
             const std::string &velocity_topic, const std::string &posture_topic,
             const TransferConfig &_config);

    void handle_datas(const std::vector<uint8_t> &datas);
    void pub_referee_msg(const RefereeData &referee_info);
    void pub_chassis_msg(const ChassisData &chassis_info);

    // 由节点的发送定时器周期调用：组 0x20 底盘控制包并发送
    void send_chassis_control();
};
