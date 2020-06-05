#include <petra_drivers/Screen.h>

Screen::Screen() : Node("Screen"), Component("Screen")
{
    display_string_subscription_ = create_subscription<std_msgs::msg::String>("DisplayString", 10, std::bind(&Screen::display_string_callback_, this, std::placeholders::_1));
    get_input_service_ = create_service<GetInput>("GetInput", std::bind(&Screen::get_input_callback_, this, std::placeholders::_1, std::placeholders::_2));
}

void Screen::display_string_callback_(const std_msgs::msg::String::SharedPtr msg)
{
    display_(msg->data);
}

void Screen::get_input_callback_(const std::shared_ptr<GetInput::Request> request, std::shared_ptr<GetInput::Response> response)
{
    display_(request->text);

    std::cin >> response->input;
    
    RCLCPP_INFO(get_logger(), "sending back response: [%s]", response->input.c_str());
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