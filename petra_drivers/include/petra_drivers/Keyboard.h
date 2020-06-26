/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#pragma once

#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

#include <petra_core/srv/get_input.hpp>

using GetInput = petra_core::srv::GetInput;

class Keyboard : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr input_string_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_publisher_;
    rclcpp::Service<GetInput>::SharedPtr get_input_service_;

    void get_input_callback_(const std::shared_ptr<GetInput::Request> request, std::shared_ptr<GetInput::Response> response);

public:
    Keyboard();

    void io_thread();
};