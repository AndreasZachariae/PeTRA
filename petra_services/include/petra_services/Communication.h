/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#pragma once

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <petra_core/default.h>
#include <petra_core/srv/user_dialog.hpp>

using UserDialog = petra_core::srv::UserDialog;

class Communication : public rclcpp::Node
{
public:
    Communication();

private:
    rclcpp::callback_group::CallbackGroup::SharedPtr input_callback_group_;

    void input_string_callback_(const std_msgs::msg::String::SharedPtr msg);
    void stop_callback_(const std_msgs::msg::Empty::SharedPtr msg);
    void dialog_callback(const std::shared_ptr<UserDialog::Request> request, std::shared_ptr<UserDialog::Response> response);

    void publish_text_(std::string);

    std::string remove_zeros_(std::string float_str);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr display_string_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr input_string_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
    rclcpp::Service<UserDialog>::SharedPtr dialog_service_;

    std::vector<std::string> input_buffer_;

    bool stop_flipflop_ = false;
};