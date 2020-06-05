#include <petra_services/CommunicationOld.h>

using namespace std;

CommunicationOld::CommunicationOld() : Node("Communication")
{
    input_callback_group_ = create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    auto input_opt = rclcpp::SubscriptionOptions();
    input_opt.callback_group = input_callback_group_;

    display_text_publisher_ = create_publisher<petra_core::msg::DisplayText>("DisplayText", 10);
    selection_service_ = create_service<Selection>("Selection", std::bind(&CommunicationOld::selection_callback, this, _1, _2));
    request_data_service_ = create_service<RequestData>("RequestData", std::bind(&CommunicationOld::request_data_callback, this, _1, _2));
    input_text_subscription_ = create_subscription<petra_core::msg::InputText>("InputText", 10, std::bind(&CommunicationOld::input_text_callback, this, _1), input_opt);
    get_input_client_ = create_client<petra_core::srv::GetInput>("GetInput", rmw_qos_profile_services_default, input_callback_group_);
    stop_subscription_ = create_subscription<std_msgs::msg::Empty>("Stop", 10, std::bind(&CommunicationOld::stop_callback, this, _1), input_opt);
}

void CommunicationOld::selection_callback(const std::shared_ptr<Selection::Request> request, std::shared_ptr<Selection::Response> response)
{
    bool stop = !stop_flipflop_;

    string msg = request->msg;
    string options = "";
    for (unsigned int i = 0; i < request->options.size(); i++)
    {
        options = options + "[" + to_string(i + 1) + "]" + request->options.at(i) + " ";
    }
    RCLCPP_INFO(get_logger(), "Incoming request. Waiting for selection...");
    publish_text(msg + "\n" + options);

    /*
    //create inner service call to get input back.

    auto inner_request = std::make_shared<petra_core::srv::GetInput::Request>();
    inner_request->text = "[inner service] Write number to choose here >> ";

    while (!get_input_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting. //kein return 0 mehr im Code");
        }
        RCLCPP_INFO(get_logger(), "service not available, waiting again...");
    }
    

    int choice = -1;
    auto inner_client_callback = [&,this](rclcpp::Client<petra_core::srv::GetInput>::SharedFuture inner_future)
        { 
            auto result = inner_future.get();
            choice = stoi(result->input);
            RCLCPP_INFO(get_logger(), "[inner service] callback executed");
        };
    auto inner_future_result = get_input_client_->async_send_request(inner_request, inner_client_callback);
    

    // Wait for the result.
    while (choice < 0 && rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    //sending back response
    response->selected = request->options.at(choice - 1);
    RCLCPP_INFO(get_logger(), "Sending back response: [%s]", response->selected.c_str());

    //end inner service call
    */

    //get input from input_text_callback with topic from Keyboard
    do
    {
        unsigned int size = input_buffer.size();
        //input_text_callback fügt dem vector ein element hinzu
        while (input_buffer.size() == size && rclcpp::ok() && (stop_flipflop_ != stop))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (stop_flipflop_ == stop)
        {
            break;
        }

        string input = input_buffer.back();
        input_buffer.pop_back();

        if (string_is_number(input)) //response typ zu int ändern oder überprüfung auslagern nach ActionSelection
        {
            int index = stoi(input);
            if ((unsigned int)index <= request->options.size() && index > 0)
            {
                response->selected = request->options.at(index - 1);
                RCLCPP_INFO(get_logger(), "Sending back response: [%s]", response->selected.c_str());
                break;
            }
            else
            {
                publish_text("Invalid Input! Number out of range. Try again");
            }
        }
        else
        {
            publish_text("Invalid Input! Number required. Try again");
        }
    } while (rclcpp::ok() && (stop_flipflop_ != stop));
}

void CommunicationOld::request_data_callback(const std::shared_ptr<RequestData::Request> request, std::shared_ptr<RequestData::Response> response)
{
    bool stop = !stop_flipflop_;

    string msg = request->msg;
    int number_of_inputs = request->number_of_inputs;
    vector<string> data_keys = request->data_keys;
    vector<string> data_values;

    RCLCPP_INFO(get_logger(), "Incoming request. Waiting for input data...");

    publish_text(msg);

    for (int i = 0; i < number_of_inputs; i++)
    {
        publish_text("[" + to_string(i + 1) + "]" + request->data_keys.at(i) + " ");

        bool exit_flag = false;
        do
        {
            unsigned int size = input_buffer.size();
            //input_text_callback fügt dem vector ein element hinzu
            while (input_buffer.size() == size && rclcpp::ok() && (stop_flipflop_ != stop))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            if (stop_flipflop_ == stop)
            {
                break;
            }

            string input = input_buffer.back();
            input_buffer.pop_back();

            if (string_is_number(input)) //check auf integer values verschieben zu RobotMovement::request_goal_
            {
                data_values.push_back(input);
                exit_flag = true;
            }
            else
            {
                publish_text("Invalid Input! Number required. Try again");
            }
        } while (!exit_flag && rclcpp::ok() && (stop_flipflop_ != stop));

        if (stop_flipflop_ == stop)
        {
            break;
        }
    }
    response->data_values.assign(data_values.begin(), data_values.end());
    RCLCPP_INFO(get_logger(), "Sending back response");
}

void CommunicationOld::publish_text(string text)
{
    auto message = petra_core::msg::DisplayText();
    message.text = text;
    //RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.text.c_str());
    display_text_publisher_->publish(message);
}

void CommunicationOld::input_text_callback(const petra_core::msg::InputText::SharedPtr msg)
{
    //RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->text.c_str());
    input_buffer.push_back(msg->text);
}

void CommunicationOld::stop_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    stop_flipflop_ = !stop_flipflop_;
    RCLCPP_WARN(get_logger(), "Stop recieved, resetting service...");
}

bool CommunicationOld::string_is_number(string string)
{
    if (string == "")
    {
        return false;
    }
    const char *str = string.c_str();
    while (*str)
    {
        if (!isdigit(*str))
        {
            return false;
        }
        ++str;
    }
    return true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<CommunicationOld>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}