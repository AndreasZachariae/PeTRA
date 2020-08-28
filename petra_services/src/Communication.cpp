#include <petra_services/Communication.h>

Communication::Communication() : Node("Communication")
{
    display_string_publisher_ = create_publisher<std_msgs::msg::String>("DisplayString", 10);

    input_options_.callback_group = create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    input_string_subscription_ = create_subscription<std_msgs::msg::String>(
        "InputString", 10, [&](const std_msgs::msg::String::SharedPtr msg) {
            //RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
            input_buffer_.push_back(msg->data);
        },
        input_options_);

    stop_subscription_ = create_subscription<std_msgs::msg::Empty>(
        "Stop", 10, [&](const std_msgs::msg::Empty::SharedPtr) {
            stop_recieved_ = true;
            RCLCPP_WARN(get_logger(), "Stop recieved, resetting service...");
        },
        input_options_);

    dialog_service_ = create_service<UserDialog>(
        "UserDialog", [&](const std::shared_ptr<UserDialog::Request> request, std::shared_ptr<UserDialog::Response> response) {
            stop_recieved_ = false;

            RCLCPP_INFO(get_logger(), "Incoming request. Waiting for input data...");

            print_header_(request);

            for (unsigned int i = 0; (i < request->data_keys.size()) && rclcpp::ok() && !stop_recieved_; i++)
            {
                petra_core::msg::DialogDataType data = request->data_keys.at(i);

                print_info_(data);

                std::string min = data.min;
                std::string max = data.max;
                std::string default_value = data.default_value;

                while (rclcpp::ok())
                {
                    input_buffer_.clear();

                    while ((input_buffer_.size() == 0) && rclcpp::ok())
                    {
                        if (stop_recieved_)
                        {
                            return;
                        }

                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }

                    switch (data.type)
                    {
                    case petra_core::msg::DialogDataType::BOOL:
                        if (input_buffer_.back() == "y")
                        {
                            input_buffer_.back() = "true";
                        }
                        else if (input_buffer_.back() == "n")
                        {
                            input_buffer_.back() = "false";
                        }
                        else
                        {
                            publish_text_("Invalid Input! Try again");
                            continue;
                        }
                        break;
                    case petra_core::msg::DialogDataType::INT:
                        int int_input;
                        try
                        {
                            int_input = std::stoi(input_buffer_.back());
                            input_buffer_.back() = std::to_string(int_input);
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

                        try
                        {
                            if ((int_input < std::stoi(min)) | (int_input > std::stoi(max)))
                            {
                                publish_text_("Number out of range! Try again");
                                continue;
                            }
                        }
                        catch (const std::invalid_argument &e)
                        {
                        }
                        break;
                    case petra_core::msg::DialogDataType::FLOAT:
                        float float_input;
                        try
                        {
                            float_input = std::stof(input_buffer_.back());
                            input_buffer_.back() = std::to_string(float_input);
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

                        try
                        {
                            if ((float_input < std::stof(min)) | (float_input > std::stof(max)))
                            {
                                publish_text_("Number out of range! Try again");
                                continue;
                            }
                        }
                        catch (const std::invalid_argument &e)
                        {
                        }
                        break;
                    case petra_core::msg::DialogDataType::STRING:
                        try
                        {
                            if ((input_buffer_.at(i).size() < (std::size_t)std::stoi(min)) | (input_buffer_.at(i).size() > (std::size_t)std::stoi(max)))
                            {
                                publish_text_("Number of characters not in range! Try again");
                                continue;
                            }
                        }
                        catch (const std::invalid_argument &e)
                        {
                        }
                        break;
                    default:
                        break;
                    }
                    break; //input is valid

                    if (stop_recieved_)
                    {
                        return;
                    }
                }

                response->data_values.push_back(input_buffer_.back());
                RCLCPP_INFO(get_logger(), "Recieved data value: [%s]", input_buffer_.back().c_str());
                input_buffer_.pop_back();
            }
        });
}

void Communication::publish_text_(std::string text)
{
    auto message = std_msgs::msg::String();
    message.data = text;

    //RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str());
    display_string_publisher_->publish(message);
}

void Communication::print_header_(const std::shared_ptr<UserDialog::Request> request)
{
    if (request->title != "")
    {
        std::string dialog_msg = ">> " + request->title + " <<";

        switch (request->importance)
        {
        case UserDialog::Request::HIGH:
            dialog_msg = "[WARNING] " + dialog_msg;
            break;
        case UserDialog::Request::CRITICAL:
            dialog_msg = "[CRITICAL] " + dialog_msg;
            break;
        }

        publish_text_(dialog_msg);
    }

    if (request->msg != "")
    {
        publish_text_(request->msg);
    }
}

void Communication::print_info_(const petra_core::msg::DialogDataType &data)
{
    std::string min = data.min;
    std::string max = data.max;
    std::string default_value = data.default_value;

    std::string type_info = "[";

    if (data.key != "")
    {
        type_info += data.key + ": ";
    }

    switch (data.type)
    {
    case petra_core::msg::DialogDataType::BOOL:
        type_info += "bool, y or n";

        if (default_value != "")
        {
            type_info += ", default = " + default_value;
        }
        break;
    case petra_core::msg::DialogDataType::INT:
        type_info += "int";

        if (min != "" && max != "")
        {
            type_info += ", between " + min + " and " + max;
        }

        if (default_value != "")
        {
            type_info += ", default = " + default_value;
        }
        break;
    case petra_core::msg::DialogDataType::FLOAT:
        type_info += "float";

        if (min != "" && max != "")
        {
            type_info += ", between " + remove_zeros_(min) + " and " + remove_zeros_(max);
        }

        if (default_value != "")
        {
            type_info += ", default = " + remove_zeros_(default_value);
        }
        break;
    case petra_core::msg::DialogDataType::STRING:
        type_info += "string";

        if (min != "" && max != "")
        {
            type_info += ", with length between " + min + " and " + max;
        }

        if (default_value != "")
        {
            type_info += ", default = " + default_value;
        }
        break;
    default:
        break;
    }

    publish_text_(type_info + "]");
}

std::string Communication::remove_zeros_(std::string float_str)
{
    if (float_str.find(".") != std::string::npos)
    {
        float_str.erase(float_str.find_last_not_of('0') + 1, std::string::npos);
    }

    float_str.erase(float_str.find_last_not_of('.') + 1, std::string::npos);
    return float_str;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<Communication> node = std::make_shared<Communication>();

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}