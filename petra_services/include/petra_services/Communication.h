/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "Communication"
 * Purpose : Provides ROS2-Service server "UserDialog"
 *           and checks input for correct type and range.
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr display_string_publisher_;
    rclcpp::SubscriptionOptions input_options_ = rclcpp::SubscriptionOptions();
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr input_string_subscription_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
    rclcpp::Service<UserDialog>::SharedPtr dialog_service_;

    std::vector<std::string> input_buffer_;

    bool stop_recieved_ = false;

    void publish_text_(std::string);

    void print_header_(const std::shared_ptr<UserDialog::Request> request);
    void print_info_(const petra_core::msg::DialogDataType &data);

    std::string remove_zeros_(std::string float_str);
};