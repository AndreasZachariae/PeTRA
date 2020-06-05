#include <petra_services/Communication.h>

Communication::Communication() : Node("Communication")
{
    input_callback_group_ = create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    auto input_opt = rclcpp::SubscriptionOptions();
    input_opt.callback_group = input_callback_group_;

    display_string_publisher_ = create_publisher<std_msgs::msg::String>("DisplayString", 10);

    input_string_subscription_ = create_subscription<std_msgs::msg::String>("InputString", 10,
                                    std::bind(&Communication::input_string_callback_, this, std::placeholders::_1), input_opt);

    get_input_client_ = create_client<petra_core::srv::GetInput>("GetInput", rmw_qos_profile_services_default, input_callback_group_);

    stop_subscription_ = create_subscription<std_msgs::msg::Empty>("Stop", 10,
                            std::bind(&Communication::stop_callback_, this, std::placeholders::_1), input_opt);

    dialogue_service_ = create_service<Dialogue>("Dialogue", 
                        std::bind(&Communication::dialogue_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void Communication::publish_text_(std::string text)
{
    auto message = std_msgs::msg::String();
    message.data = text;

    //RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
    display_string_publisher_->publish(message);
}

void Communication::input_string_callback_(const std_msgs::msg::String::SharedPtr msg)
{
    //RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
    input_buffer_.push_back(msg->data);
}

void Communication::stop_callback_(const std_msgs::msg::Empty::SharedPtr msg)
{
    stop_flipflop_ = !stop_flipflop_;
    RCLCPP_WARN(get_logger(), "Stop recieved, resetting service...");
}

void Communication::dialogue_callback(const std::shared_ptr<Dialogue::Request> request, std::shared_ptr<Dialogue::Response> response)
{
    bool stop = !stop_flipflop_;

    RCLCPP_INFO(get_logger(), "Incoming request. Waiting for input data...");
    std::string dialogue_msg;

    switch (request->importance)
    {
    case Dialogue::Request::NORMAL:
        dialogue_msg = "";
        break;
    case Dialogue::Request::HIGH:
        dialogue_msg = "[WARNING] ";
        break;
    case Dialogue::Request::CRITICAL:
        dialogue_msg = "[CRITICAL] ";
        break;
    default:
        break;
    }
    dialogue_msg += ">> " + request->title + " <<\n" + request->msg;
    publish_text_(dialogue_msg);

    for (unsigned int i = 0; (i < request->data_keys.size()) && rclcpp::ok() && (stop_flipflop_ != stop); i++)
    {
        std::string min = request->data_keys.at(i).min;
        std::string max = request->data_keys.at(i).max;
        std::string default_value = request->data_keys.at(i).default_value;

    std::string type_info = "[" + request->data_keys.at(i).key + ": ";

        switch (request->data_keys.at(i).type)
        {
        case petra_core::msg::DialogueDataType::INT:
            type_info += "int, " + min + " - " + max + ", default = " + default_value + "]";
            break;
        case petra_core::msg::DialogueDataType::BOOL:
            type_info += "bool, " + min + "/" + max + " or y/n , default = " + default_value + "]";
            break;
        case petra_core::msg::DialogueDataType::FLOAT:
            type_info += "float, " + remove_zeros_(min) + " - " + remove_zeros_(max) + ", default = " + remove_zeros_(default_value) + "]";
            break;
        case petra_core::msg::DialogueDataType::STRING:
            type_info += "string, " + min + " - " + max + " long, default = " + default_value + "]";
            break;
        default:
            break;
        }
        publish_text_(type_info);

        do
        {
            input_buffer_.clear();
            //input_string_callback_ fügt dem vector ein element hinzu
            while ((input_buffer_.size() == 0) && rclcpp::ok() && (stop_flipflop_ != stop))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            if (stop_flipflop_ == stop)
            {
                break;
            }

            switch (request->data_keys.at(i).type)
            {
            case petra_core::msg::DialogueDataType::INT:
                int int_input;
                try
                {
                    int_input = std::stoi(input_buffer_.back());
                }
                catch (const std::invalid_argument &e)
                {
                    publish_text_("Invalid Input! Try again");
                    continue;
                }
                catch (const std::out_of_range &e)
                {
                    publish_text_("Number out of range! Try again");
                    continue;
                }
                if ((int_input < std::stoi(min)) | (int_input > std::stoi(max)))
                {
                    publish_text_("Number out of range! Try again");
                    continue;
                }
                break;
            case petra_core::msg::DialogueDataType::BOOL:
                if ((input_buffer_.back() == "0") | (input_buffer_.back() == "1") | (input_buffer_.back() == "y") | (input_buffer_.back() == "n"))
                {
                }
                else
                {
                    continue;
                }
                break;
            case petra_core::msg::DialogueDataType::FLOAT:
                float float_input;
                try
                {
                    float_input = std::stof(input_buffer_.back());
                }
                catch (const std::invalid_argument &e)
                {
                    publish_text_("Invalid Input! Try again");
                    continue;
                }
                catch (const std::out_of_range &e)
                {
                    publish_text_("Number out of range! Try again");
                    continue;
                }
                if ((float_input < std::stof(min)) | (float_input > std::stof(max)))
                {
                    publish_text_("Number out of range! Try again");
                    continue;
                }
                break;
            case petra_core::msg::DialogueDataType::STRING:
                if ((input_buffer_.at(i).size() < std::stoi(min)) | (input_buffer_.at(i).size() > std::stoi(max)))
                {
                    publish_text_("Number of characters not in range! Try again");
                    continue;
                }
                break;
            default:
                break;
            }
            break; //input is valid

        } while (rclcpp::ok() && (stop_flipflop_ != stop));

        if (stop_flipflop_ == stop)
        {
            break;
        }

        response->data_values.push_back(input_buffer_.back());
        RCLCPP_INFO(get_logger(), "Recieved data value: [%s]", input_buffer_.back().c_str());
        input_buffer_.pop_back();
    }
}

std::string Communication::remove_zeros_(std::string float_str)
{
    float_str.erase(float_str.find_last_not_of('0') + 1, std::string::npos);
    float_str.erase(float_str.find_last_not_of('.') + 1, std::string::npos);
    return float_str;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<Communication>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}