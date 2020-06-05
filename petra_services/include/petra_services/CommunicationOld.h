/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>

#include <petra_core/default.h>
#include <petra_core/srv/selection.hpp>
#include <petra_core/srv/get_input.hpp>
#include <petra_core/msg/display_text.hpp>
#include <petra_core/msg/input_text.hpp>
#include <petra_core/srv/request_data.hpp>

#include <functional>
#include <memory>
#include <queue>
#include <thread>
#include <chrono>

using Selection = petra_core::srv::Selection;
using RequestData = petra_core::srv::RequestData;
using namespace std::placeholders;

class CommunicationOld : public rclcpp::Node
{
public:
    CommunicationOld();

    void publish_text(std::string);
    bool string_is_number(std::string string);

private:
    rclcpp::callback_group::CallbackGroup::SharedPtr input_callback_group_;

    void selection_callback(const std::shared_ptr<Selection::Request> request, std::shared_ptr<Selection::Response> response);
    void request_data_callback(const std::shared_ptr<RequestData::Request> request, std::shared_ptr<RequestData::Response> response);
    void input_text_callback(const petra_core::msg::InputText::SharedPtr msg);
    void stop_callback(const std_msgs::msg::Empty::SharedPtr msg);

    rclcpp::Publisher<petra_core::msg::DisplayText>::SharedPtr display_text_publisher_;
    rclcpp::Service<Selection>::SharedPtr selection_service_;
    rclcpp::Service<RequestData>::SharedPtr request_data_service_;
    rclcpp::Subscription<petra_core::msg::InputText>::SharedPtr input_text_subscription_;
    rclcpp::Client<petra_core::srv::GetInput>::SharedPtr get_input_client_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
    std::vector<std::string> input_buffer;

    bool stop_flipflop_ = false;
};