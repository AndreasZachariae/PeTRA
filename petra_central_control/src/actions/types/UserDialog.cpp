#include <petra_central_control/actions/types/UserDialog.h>
#include <petra_central_control/actions/base/RosNode.h>

UserDialog::UserDialog(std::string title, std::string msg, uint8_t importance) : Component(title)
{
    request_ = std::make_shared<petra_core::srv::UserDialog::Request>();
    request_->title = title;
    request_->msg = msg;
    request_->importance = importance;
    dialog_client_ = RosNode::get_node_handle()->create_client<petra_core::srv::UserDialog>("UserDialog");

    display_string_publisher_ = RosNode::get_node_handle()->create_publisher<std_msgs::msg::String>("DisplayString", 10);
}

void UserDialog::add_int_key(std::string key, int min, int max, int default_value)
{
    add_key_(petra_core::msg::DialogDataType::INT, key, std::to_string(min), std::to_string(max), std::to_string(default_value));
}

void UserDialog::add_bool_key(std::string key, bool default_value)
{
    add_key_(petra_core::msg::DialogDataType::BOOL, key, "0", "1", std::to_string(default_value));
}

void UserDialog::add_float_key(std::string key, float min, float max, float default_value)
{
    add_key_(petra_core::msg::DialogDataType::FLOAT, key, std::to_string(min), std::to_string(max), std::to_string(default_value));
}

void UserDialog::add_string_key(std::string key, int min_chars, int max_chars, std::string default_value)
{
    add_key_(petra_core::msg::DialogDataType::STRING, key, std::to_string(min_chars), std::to_string(max_chars), default_value);
}

void UserDialog::add_key_(int type, std::string key, std::string min, std::string max, std::string default_value)
{
    petra_core::msg::DialogDataType data_type;

    data_type.type = type;
    data_type.key = key;
    data_type.min = min;
    data_type.max = max;
    data_type.default_value = default_value;

    request_->data_keys.push_back(data_type);
}

bool UserDialog::send_dialog(std::function<void()> callback)
{
    if (dialog_client_->service_is_ready())
    {
        response_future_ = dialog_client_->async_send_request(request_, [callback](rclcpp::Client<petra_core::srv::UserDialog>::SharedFuture future) {
            callback();
            (void)future;
        });
        return true;
    }
    else
    {
        return false;
    }
}

std::shared_ptr<petra_core::srv::UserDialog_Response> UserDialog::get_response()
{
    return response_future_.get();
}

void UserDialog::send_msg(std::string msg)
{
    auto message = std_msgs::msg::String();
    message.data = msg;

    display_string_publisher_->publish(message);
}