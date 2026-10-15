#include "transfer.h"

Transfer::Transfer(rclcpp::Node *node, std::shared_ptr<serial::Serial> &_serial_port,
                   const std::string &velocity_topic, const std::string &posture_topic,
                   const TransferConfig &_config)
    : serial_port(_serial_port), config(_config)
{
    this->node = node;

    referee_puber = node->create_publisher<RefereeMsg>("referee_data", 10);
    chassis_puber = node->create_publisher<ChassisMsg>("chassis_data", 10);

    velocity_suber = node->create_subscription<VelocityMsg>(
        velocity_topic, 10,
        std::bind(&Transfer::velocity_callback, this, std::placeholders::_1));
    posture_suber = node->create_subscription<PostureMsg>(
        posture_topic, 10,
        std::bind(&Transfer::posture_callback, this, std::placeholders::_1));
}

// 导航速度指令回调：只缓存最新值，实际发送由发送定时器统一完成
void Transfer::velocity_callback(const VelocityMsg::SharedPtr msg){
    std::lock_guard<std::mutex> lock(cmd_mutex);
    velocity_cmd = *msg;
    velocity_stamp = node->now();
    velocity_received = true;
}

// 决策姿态指令回调
void Transfer::posture_callback(const PostureMsg::SharedPtr msg){
    std::lock_guard<std::mutex> lock(cmd_mutex);
    posture_cmd = msg->posture;
    posture_stamp = node->now();
    posture_received = true;
}

void Transfer::handle_datas(const std::vector<uint8_t> &datas){
    if(datas[1] == serial_referee_id){
        RefereeData referee;
        referee = trans_uint8_struct<RefereeData>(datas, serial_referee_id, CRCMode::crc16);
        pub_referee_msg(referee);
        // info_log("recv referee info");
    }else if(datas[1] == serial_move_id) {
        ChassisData chassis;
        chassis = trans_uint8_struct<ChassisData>(datas, serial_move_id, CRCMode::crc16);
        pub_chassis_msg(chassis);
        // info_log("recv chassis data");
    }
}

void Transfer::pub_referee_msg(const RefereeData &referee_info){
    RefereeMsg msg;
    msg.game_time = referee_info.game_time;
    msg.sentry_posture = referee_info.sentry_posture;
    referee_puber->publish(msg);
}

void Transfer::pub_chassis_msg(const ChassisData &chassis_info){
    ChassisMsg msg;
    msg.vx = chassis_info.vx;
    msg.vy = chassis_info.vy;
    msg.wz = chassis_info.wz;
    chassis_puber->publish(msg);
}

void Transfer::send_chassis_control(){
    ChassisControl ctrl{};   // 占位符全部置 0
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    uint8_t posture = config.default_posture;
    bool command_lost = false;

    {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        const bool velocity_fresh = velocity_received &&
            (node->now() - velocity_stamp).seconds() <= config.velocity_timeout;
        if(velocity_fresh){
            vx = velocity_cmd.vx;
            vy = velocity_cmd.vy;
            wz = velocity_cmd.wz;
        }else{
            // 看门狗：指令超时就不再沿用旧指令，直接发零速
            command_lost = velocity_received;
        }
        // 姿态指令超时则回到默认姿态（移动姿态），避免决策掉线后卡在得分姿态
        const bool posture_fresh = posture_received &&
            (node->now() - posture_stamp).seconds() <= config.posture_timeout;
        if(posture_fresh){
            posture = posture_cmd;
        }
    }

    if(command_lost){
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
            "velocity command timeout (>%.3f s), sending zero velocity", config.velocity_timeout);
    }

    ctrl.vx = vx;
    ctrl.vy = vy;
    ctrl.wz = wz;
    ctrl.sentry_posture = posture;

    send_serial_packet(
        trans_struct_uint8<ChassisControl>(chassis_control_id, ctrl, CRCMode::crc16),
        "chassis packet");
}

bool Transfer::send_serial_packet(const std::vector<uint8_t> &datas, const std::string &tag) {
    std::lock_guard<std::mutex> lock(write_mutex);

    if (!serial_port) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
            "serial port is null, skip %s", tag.c_str());
        return false;
    }
    if (!serial_port->isOpen()) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
            "serial port is not open, skip %s", tag.c_str());
        return false;
    }

    try {
        serial_port->write(datas);
        return true;
    } catch (const std::exception &ex) {
        error_log("failed to send ", tag, ": ", ex.what());
        return false;
    }
}
