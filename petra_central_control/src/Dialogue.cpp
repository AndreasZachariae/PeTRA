#include <petra_central_control/Dialogue.h>

Dialogue::Dialogue(std::shared_ptr<rclcpp::Node> node_handle, std::string title, std::string msg, DialogueImportanceType importance) 
: Component(title), node_handle_(node_handle)
{
    dialogue_ = std::make_shared<petra_core::srv::Dialogue::Request>();
    dialogue_->title = title;
    dialogue_->msg = msg;
    dialogue_->importance = importance;
    dialogue_client_ = node_handle_->create_client<petra_core::srv::Dialogue>("Dialogue");
}

void Dialogue::add_int_key(std::string key, int min, int max, int default_value)
{
    add_key_(petra_core::msg::DialogueDataType::INT, key, std::to_string(min), std::to_string(max), std::to_string(default_value));
}

void Dialogue::add_bool_key(std::string key, bool default_value)
{
    add_key_(petra_core::msg::DialogueDataType::BOOL, key, "0", "1", std::to_string(default_value));
}

void Dialogue::add_float_key(std::string key, float min, float max, float default_value)
{
    add_key_(petra_core::msg::DialogueDataType::FLOAT, key, std::to_string(min), std::to_string(max), std::to_string(default_value));
}

void Dialogue::add_string_key(std::string key, int min_chars, int max_chars, std::string default_value)
{
    add_key_(petra_core::msg::DialogueDataType::STRING, key, std::to_string(min_chars), std::to_string(max_chars), default_value);
}

void Dialogue::add_key_(int type, std::string key, std::string min, std::string max, std::string default_value)
{
    petra_core::msg::DialogueDataType data_type;
    
    data_type.type = type;
    data_type.key = key;
    data_type.min = min;
    data_type.max = max;
    data_type.default_value = default_value;

    dialogue_->data_keys.push_back(data_type);
}

bool Dialogue::send_dialogue(std::function<void()> callback)
{
    if (dialogue_client_->service_is_ready())
    {
        response_future_ = dialogue_client_->async_send_request(dialogue_, [callback](rclcpp::Client<petra_core::srv::Dialogue>::SharedFuture future) 
        {
            callback();
        });
        return true;
    }
    else
    {
        return false;
    }
}

std::shared_ptr<petra_core::srv::Dialogue_Response> Dialogue::get_response()
{
    return response_future_.get();
}