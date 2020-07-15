#pragma once

#include <petra_central_control/default.h>

#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <petra_core/srv/user_dialog.hpp>

class UserDialog : public Component
{
public:
    UserDialog() {}
    UserDialog(std::string title, std::string msg = "", uint8_t importance = petra_core::srv::UserDialog::Request::NORMAL);

    void add_int_key(std::string key, int min, int max, int default_value = petra_core::msg::DialogDataType::INT_DEFAULT);
    void add_bool_key(std::string key, bool default_value = petra_core::msg::DialogDataType::BOOL_DEFAULT);
    void add_float_key(std::string key, float min, float max, float default_value = petra_core::msg::DialogDataType::FLOAT_DEFAULT);
    void add_string_key(std::string key, int min_chars, int max_chars, std::string default_value = petra_core::msg::DialogDataType::STRING_DEFAULT);

    bool send_dialog(std::function<void()> callback);
    void send_msg(std::string msg);

    std::shared_ptr<petra_core::srv::UserDialog_Response> get_response();

private:
    rclcpp::Client<petra_core::srv::UserDialog>::SharedPtr dialog_client_;
    std::shared_ptr<petra_core::srv::UserDialog_Request> request_;
    std::shared_future<std::shared_ptr<petra_core::srv::UserDialog_Response>> response_future_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr display_string_publisher_;

    void add_key_(int type, std::string key, std::string min, std::string max, std::string default_value);
};