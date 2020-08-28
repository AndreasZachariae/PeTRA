/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "Keyboard"
 * Purpose : Reads keyboard inputs from the terminal and 
 *           pubishes them in topic "InputString".
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

#include <petra_core/default.h>

class Keyboard : public rclcpp::Node
{
public:
    Keyboard();

    void io_thread();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr input_string_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_publisher_;
};