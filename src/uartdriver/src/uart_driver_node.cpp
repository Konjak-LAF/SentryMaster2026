#include "uart_driver_node.h"
#include <algorithm>
#include <chrono>

UARTDriverNode::UARTDriverNode(std::string name) : Node(name){
    init_node();
    info_log(name, "has been built!");
}

void UARTDriverNode::init_node(){
    // 串口参数
    declare_parameter("serial_number", "");
    serial_number_filter = get_parameter("serial_number").as_string();

    declare_parameter("serial_name", "");
    serial_name_fallback = get_parameter("serial_name").as_string();

    declare_parameter("baud_rate", 0);
    baud_rate = get_parameter("baud_rate").as_int();

    declare_parameter("read_buffer_size", 0);
    read_buffer_size = get_parameter("read_buffer_size").as_int();

    declare_parameter("possible_ids", std::vector<int>());
    auto int_ids = get_parameter("possible_ids").as_integer_array();

    // 发送链路参数
    declare_parameter("velocity_command_topic", "/sentry/velocity_command");
    std::string velocity_topic = get_parameter("velocity_command_topic").as_string();

    declare_parameter("posture_command_topic", "/sentry/posture_command");
    std::string posture_topic = get_parameter("posture_command_topic").as_string();

    declare_parameter("send_rate_hz", 50.0);
    double send_rate_hz = get_parameter("send_rate_hz").as_double();

    declare_parameter("velocity_timeout", 0.2);
    double velocity_timeout = get_parameter("velocity_timeout").as_double();

    declare_parameter("posture_timeout", 1.0);
    double posture_timeout = get_parameter("posture_timeout").as_double();

    declare_parameter("default_posture", 3);
    int default_posture = get_parameter("default_posture").as_int();

    log_parameters(this, {"serial_number", "serial_name", "baud_rate", "read_buffer_size",
                          "possible_ids", "velocity_command_topic", "posture_command_topic",
                          "send_rate_hz", "velocity_timeout", "posture_timeout",
                          "default_posture"});

    if(read_buffer_size < 32){
        error_log("read_buffer_size", read_buffer_size, "is too small, use 60 instead");
        read_buffer_size = 60;
    }
    if(send_rate_hz <= 0.0){
        error_log("send_rate_hz", send_rate_hz, "is invalid, use 50.0 instead");
        send_rate_hz = 50.0;
    }
    if(velocity_timeout <= 0.0){
        error_log("velocity_timeout", velocity_timeout, "is invalid, use 0.2 instead");
        velocity_timeout = 0.2;
    }
    if(posture_timeout <= 0.0){
        error_log("posture_timeout", posture_timeout, "is invalid, use 1.0 instead");
        posture_timeout = 1.0;
    }

    for (auto int_id : int_ids){
        uint8_t id = static_cast<uint8_t>(int_id);
        possible_ids.push_back(id);
        // info_log((int)id);
    }

    // 打开串口失败不致命：读线程会周期性重连，保证无硬件时节点仍能启动
    port_mutex = std::make_shared<std::mutex>();
    if(!try_open_serial()){
        error_log("failed to open serial port, will keep retrying in background");
    }

    TransferConfig transfer_config;
    transfer_config.velocity_timeout = velocity_timeout;
    transfer_config.posture_timeout = posture_timeout;
    transfer_config.default_posture = static_cast<uint8_t>(default_posture);

    uart_transfer = std::make_shared<Transfer>(this, serial_port, port_mutex,
                                               velocity_topic, posture_topic, transfer_config);
    read_thread = std::thread(&UARTDriverNode::read_from_port, this);
    start_send_timer(send_rate_hz);

    info_log("serial", serial_name, "baud_rate", baud_rate,
             "send_rate_hz", send_rate_hz);
}

bool UARTDriverNode::try_open_serial(){
    std::string port_name = serial_name_fallback;

    // serial_number_filter 为空白串时直接使用 yaml 中写死的端口名，避免误匹配其它设备
    const bool has_serial_number =
        serial_number_filter.find_first_not_of(" \t\n") != std::string::npos;
    if(has_serial_number){
        try{
            for (const auto &port : serial::list_ports()) {
                if (port.hardware_id.find(serial_number_filter) != std::string::npos) {
                    port_name = port.port;
                    info_log("match port name: ", port_name);
                    break;
                }
            }
        }catch (const std::exception &ex){
            error_log("failed to list serial ports: ", ex.what());
        }
    }

    if(port_name.empty()){
        error_log("no serial port name configured (serial_name is empty)");
        return false;
    }

    // 与发送线程的 write 互斥：替换句柄时不能让发送端拿到半开状态的端口
    std::lock_guard<std::mutex> lock(*port_mutex);

    try{
        auto port = std::make_shared<serial::Serial>(
            port_name, baud_rate, serial::Timeout::simpleTimeout(1000));
        if(!port->isOpen()){
            port->open();
        }
        serial_port = port;
        serial_name = port_name;
        is_serial_normal = serial_port->isOpen();
        if(is_serial_normal){
            info_log("open", serial_name, "in", baud_rate);
        }
        return is_serial_normal;
    }catch (const std::exception &ex){
        serial_port = nullptr;
        is_serial_normal = false;
        error_log("Error creating serial port:", port_name, ex.what());
        return false;
    }
}

void UARTDriverNode::start_send_timer(double send_rate_hz){
    send_timer = create_wall_timer(
        std::chrono::duration<double>(1.0 / send_rate_hz),
        [this](){ uart_transfer->send_chassis_control(); });
}

void UARTDriverNode::read_from_port() {
    std::vector<uint8_t> read_buffer(read_buffer_size);
    uint8_t *buffer = read_buffer.data();

    int no_serial_data = 0; 
    int read_data_size = 0;

    while (rclcpp::ok()) {
        // 串口未就绪：周期性重连（单次尝试，失败就等下一轮，不做递归重试）
        if (!is_serial_normal) {
            if(!reopen_port()){
                rclcpp::sleep_for(std::chrono::seconds(2));
            }
            continue;
        }

        try {
            serial_port->read(buffer, 1);
            if (buffer[0] == serial_info_start) {
                read_data_size = 1;
                serial_port->read(buffer + 1, 1);

                if (std::find(possible_ids.begin(), possible_ids.end(), buffer[1]) != possible_ids.end()) {
                    uint8_t *read_end = buffer + 1;
                    read_data_size++;
                    bool have_convert = false;

                    while ((have_convert || read_end[0] != serial_info_end) && read_data_size < read_buffer_size) {
                        have_convert = false;
                        ++read_end;
                        serial_port->read(read_end, 1);

                        if (read_end[0] == 0x7f) {
                            serial_port->read(read_end, 1);
                            read_end[0] = read_end[0] + 0x7d;
                            have_convert = true;
                        }
                        ++read_data_size;
                    }

                    if (read_end[0] == serial_info_end) {
                        std::vector<uint8_t> data;
                        data.reserve(read_data_size);
                        for (int i = 0; i < read_data_size; i++) {
                            data.push_back(read_buffer[i]);
                        }
                        uart_transfer->handle_datas(data);
                        no_serial_data = 0;
                        continue;
                    }
                }
                // 解析出来start但是没有数据
                no_serial_data++;
            }

            if (no_serial_data > 5) {
                error_log("no serial data....");
                no_serial_data = 0;
                close_serial();
            }
        } catch (const std::exception &ex) {
            error_log("Error while receiving data: ", ex.what());
            close_serial();
        }
    }
}

// 关闭并释放串口句柄（持锁，避免打断发送线程正在进行的 write）
void UARTDriverNode::close_serial(){
    std::lock_guard<std::mutex> lock(*port_mutex);
    is_serial_normal = false;
    if (serial_port && serial_port->isOpen()) {
        try{
            serial_port->close();
        }catch (const std::exception &ex){
            error_log("failed to close serial port: ", ex.what());
        }
    }
    serial_port = nullptr;
}

bool UARTDriverNode::reopen_port(){
    close_serial();

    if(!try_open_serial()){
        error_log("Attempting to reopen", serial_name, "failed, retry later");
        return false;
    }
    info_log("Successfully reopened", serial_name);
    return true;
}
