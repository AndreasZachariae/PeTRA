/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <petra_core/Component.h>

class Screen : public rclcpp::Node, public Component
{
public:
    Screen();

    void display_(std::string);

private:
    void display_string_callback_(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr display_string_subscription_;
};