#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2/utils.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/odometry.hpp>
using OdomMsg = nav_msgs::msg::Odometry;

#include "rm_msgs/msg/chassis_data.hpp"
using ChassisDataMsg = rm_msgs::msg::ChassisData;

#include "rm_msgs/msg/nav_command.hpp"
using NavCommandMsg = rm_msgs::msg::NavCommand;

#include "rm_msgs/msg/velocity_command.hpp"
using VelocityCommandMsg = rm_msgs::msg::VelocityCommand;

inline void info_log() {std::cout << "\33[0m" << std::endl;}
// 发布绿色信息
template<typename T, typename... Args>
void info_log(T element, Args... args) {
    std::cout << "\33[32m" << element << " "; 
    info_log(args...);         
}

inline void error_log() {std::cout << "\33[0m" << std::endl;}
// 发布红色信息
template<typename T, typename... Args>
void error_log(T element, Args... args) {
    std::cout << "\33[31m" << element << " "; 
    error_log(args...);         
}

// 打印本次启动实际生效的参数（预检录用：确认 yaml 是否传递成功）
inline void log_parameters(rclcpp::Node *node, const std::vector<std::string> &names){
    for(const auto &name : names){
        if(!node->has_parameter(name)){
            error_log("[param]", node->get_name(), name, "is not declared!");
            continue;
        }
        info_log("[param]", node->get_name(), name, "=",
                 node->get_parameter(name).value_to_string());
    }
}
