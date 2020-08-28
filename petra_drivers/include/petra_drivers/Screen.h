/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "Screen"
 * Purpose : Displays all messages from the topic "DisplayString" in the terminal.
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <petra_core/default.h>
#include <petra_core/Component.h>

class Screen : public rclcpp::Node, public Component
{
public:
    Screen();

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr display_string_subscription_;
};