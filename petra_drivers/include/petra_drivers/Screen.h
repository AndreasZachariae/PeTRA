/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <petra_core/Component.h>
#include <petra_core/srv/get_input.hpp>

using GetInput = petra_core::srv::GetInput;

class Screen : public rclcpp::Node, public Component
{
public:
    Screen();

    void display_(std::string);

private:
    void get_input_callback_(const std::shared_ptr<GetInput::Request> request, std::shared_ptr<GetInput::Response> response);
    void display_string_callback_(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr display_string_subscription_;
    rclcpp::Service<GetInput>::SharedPtr get_input_service_;
};