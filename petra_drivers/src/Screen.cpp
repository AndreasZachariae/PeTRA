#include <petra_drivers/Screen.h>

Screen::Screen() : Node("Screen"), Component("Screen")
{
    display_string_subscription_ = create_subscription<std_msgs::msg::String>("DisplayString", 10, std::bind(&Screen::display_string_callback_, this, std::placeholders::_1));
}

void Screen::display_string_callback_(const std_msgs::msg::String::SharedPtr msg)
{
    display_(msg->data);
}

void Screen::display_(std::string msg)
{
    std::stringstream stream(msg);

    while (stream.good()){
        std::string line;
        std::getline(stream,line);
        log(line);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Screen>());
    rclcpp::shutdown();
    return 0;
}