#pragma once
#include "common.h"
#include <memory>
#include <mutex>
#include <thread>
#include "transfer.h"

// 串口驱动节点
class UARTDriverNode : public rclcpp::Node{ 
private:
    std::thread read_thread;
    std::string serial_name;          // 实际打开的端口名（可能由 USB 序列号解析得到）
    std::string serial_name_fallback; // yaml 中写死的端口名
    std::string serial_number_filter; // yaml 中的 USB 序列号（空白表示不按序列号匹配）
    std::vector<uint8_t> possible_ids;

    int baud_rate = 0;
    int read_buffer_size = 60;
    bool is_serial_normal = false;

    std::shared_ptr<Transfer> uart_transfer;
    std::shared_ptr<serial::Serial> serial_port;
    // 串口句柄互斥：write（发送定时器线程）与 close/open（读线程）之间必须互斥。
    // read 只发生在读线程、close/open 也由读线程发起，所以 read 本身不需要持锁。
    std::shared_ptr<std::mutex> port_mutex;

    // 周期发送 0x20 底盘控制包
    rclcpp::TimerBase::SharedPtr send_timer;

    void init_node();
    void read_from_port();
    // 打开串口并替换句柄，失败返回 false（不抛异常）
    bool try_open_serial();
    // 关闭并释放句柄（持锁，供读线程在异常/失联/重连时调用）
    void close_serial();
    // 单次重连尝试，成功返回 true；重试节奏由读线程控制
    bool reopen_port();
    void start_send_timer(double send_rate_hz);

public:
    UARTDriverNode(std::string name);
};
