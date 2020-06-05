#include <petra_drivers/Keyboard.h>

Keyboard::Keyboard() : Node("Keyboard")
{
    input_string_publisher_ = create_publisher<std_msgs::msg::String>("InputString", 10);
    stop_publisher_ = create_publisher<std_msgs::msg::Empty>("Stop", 10);
}

void Keyboard::io_thread()
{
    while (rclcpp::ok())
    {
        std::string input;
        std::cin >> input;
        if (input.at(0) == '/')
        {
            if (input == "/stop")
            {
                auto command = std_msgs::msg::Empty();
                RCLCPP_WARN(get_logger(), "Executing command: '%s'", input.c_str());
                stop_publisher_->publish(command);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Unknown command: '%s'", input.c_str());
            }
        }
        else
        {
            auto message = std_msgs::msg::String();
            message.data = input;
            RCLCPP_INFO(get_logger(), "Publishing: '%s'", input.c_str());
            input_string_publisher_->publish(message);
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    Keyboard key;
    std::thread{std::bind(&Keyboard::io_thread, &key)}.detach();
    std::shared_ptr<Keyboard> node(&key);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}