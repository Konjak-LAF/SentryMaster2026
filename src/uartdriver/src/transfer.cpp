#include "transfer.h"

Transfer::Transfer(rclcpp::Node *node, std::shared_ptr<serial::Serial> &_serial_port,
                   std::shared_ptr<std::mutex> _port_mutex,
                   const std::string &velocity_topic, const std::string &posture_topic,
                   const TransferConfig &_config)
    : serial_port(_serial_port), port_mutex(_port_mutex), config(_config)
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

// 接收链路：校验通过的帧才发布；校验失败的一律丢弃并计数（不发布全零假数据）
void Transfer::handle_datas(const std::vector<uint8_t> &datas){
    if(datas.size() < serial_header_size + serial_tail_size8){
        return;   // 防御性判断：正常路径只会给出带首尾标志的完整帧
    }

    if(datas[1] == serial_referee_id){
        RefereeData referee;
        if(!parse_serial_struct(datas, serial_referee_id, CRCMode::crc16, referee)){
            ++invalid_referee_frames;
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                "invalid referee frame dropped (size=%zu, dropped=%llu)",
                datas.size(), static_cast<unsigned long long>(invalid_referee_frames));
            return;
        }
        pub_referee_msg(referee);
    }else if(datas[1] == serial_move_id) {
        ChassisData chassis;
        if(!parse_serial_struct(datas, serial_move_id, CRCMode::crc16, chassis)){
            ++invalid_chassis_frames;
            RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                "invalid chassis frame dropped (size=%zu, dropped=%llu)",
                datas.size(), static_cast<unsigned long long>(invalid_chassis_frames));
            return;
        }
        pub_chassis_msg(chassis);
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
    ChassisControl ctrl{};   // 占位符默认 0；ph1 由电控要求固定为 4（见下方赋值）
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
    ctrl.ph1 = 4;                 // 电控要求：该占位字节固定为 4（2026-10-17 通知），其余占位字节保持 0
    ctrl.sentry_posture = posture;

    send_serial_packet(
        trans_struct_uint8<ChassisControl>(chassis_control_id, ctrl, CRCMode::crc16),
        "chassis packet");
}

bool Transfer::send_serial_packet(const std::vector<uint8_t> &datas, const std::string &tag) {
    // 与节点的 close/open 互斥：串口重连不会打断正在进行的 write（反之亦然）
    std::lock_guard<std::mutex> lock(*port_mutex);

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
