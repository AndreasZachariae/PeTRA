/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

#include <iostream>
#include <string>

class Keyboard : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr input_string_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_publisher_;

public:
    Keyboard();

    void io_thread();
};