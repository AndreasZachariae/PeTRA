#pragma once

#include <rclcpp/rclcpp.hpp>

#include <petra_central_control/default.h>
#include <petra_central_control/DialogueImportanceType.h>

#include <functional>

class Dialogue : public Component
{
public:
    Dialogue() {}
    Dialogue(std::shared_ptr<rclcpp::Node> node_handle, std::string title = "Dialogue", std::string msg = "", DialogueImportanceType importance = DialogueImportanceType::normal);

    void add_int_key(std::string key, int min, int max, int default_value = petra_core::msg::DialogueDataType::INT_DEFAULT);
    void add_bool_key(std::string key, bool default_value = petra_core::msg::DialogueDataType::BOOL_DEFAULT);
    void add_float_key(std::string key, float min, float max, float default_value = petra_core::msg::DialogueDataType::FLOAT_DEFAULT);
    void add_string_key(std::string key, int min_chars, int max_chars, std::string default_value = petra_core::msg::DialogueDataType::STRING_DEFAULT);

    bool send_dialogue(std::function<void()> callback);
    
    std::shared_ptr<petra_core::srv::Dialogue_Response> get_response();

private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    rclcpp::Client<petra_core::srv::Dialogue>::SharedPtr dialogue_client_;
    std::shared_ptr<petra_core::srv::Dialogue_Request> dialogue_;
    std::shared_future<std::shared_ptr<petra_core::srv::Dialogue_Response>> response_future_;

    void add_key_(int type, std::string key, std::string min, std::string max, std::string default_value);
};