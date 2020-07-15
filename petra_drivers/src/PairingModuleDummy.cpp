#include <petra_drivers/PairingModuleDummy.h>

PairingModuleDummy::PairingModuleDummy(const rclcpp::NodeOptions &options) : Node("PairingModuleDummy", options)
{
    pair_device_server_ = rclcpp_action::create_server<PairDevice>(
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        "PairDevice",
        std::bind(&PairingModuleDummy::handle_goal_, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PairingModuleDummy::handle_cancel_, this, std::placeholders::_1),
        std::bind(&PairingModuleDummy::handle_accepted_, this, std::placeholders::_1));
    
    diagnostic_status_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("DiagnosticStatus", 10);

    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&PairingModuleDummy::timer_callback_, this));
}

void PairingModuleDummy::timer_callback_()
{
    auto diagnostic = diagnostic_msgs::msg::DiagnosticStatus();
    diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostic.name = "PairingModuleDummy";
    diagnostic.hardware_id = "4";

    //RCLCPP_INFO(get_logger(), "Publishing DiagnosticStatus");
    diagnostic_status_publisher_->publish(diagnostic);
}

rclcpp_action::GoalResponse PairingModuleDummy::handle_goal_(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const PairDevice::Goal> goal)
{
    if ((goal->pairing_mode == PairDevice::Goal::PAIR) && paired_)
    {
        RCLCPP_WARN(get_logger(), "There is already a device paired!");
        return rclcpp_action::GoalResponse::REJECT;
    } 
    else if ((goal->pairing_mode == PairDevice::Goal::UNPAIR) && !paired_)
    {
        RCLCPP_WARN(get_logger(), "There is no device to unpair!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    std::string msg;

    if (goal->pairing_mode == PairDevice::Goal::PAIR)
    {
        msg = "Pair";
    } 
    else
    {
        msg = "Unpair";
    }

    switch (goal->device_type)
    {
    case PairDevice::Goal::WHEELCHAIR:
        msg = msg + " Wheelchair";
        break;
    case PairDevice::Goal::ROLLATOR:
        msg = msg + " Rollator";
        break;
    case PairDevice::Goal::HOSPITALBED:
        msg = msg + " HospitalBed";
        break;
    case PairDevice::Goal::CHARGINGSTATION:
        msg = msg + " ChargingStation";
        break;
    
    default:
        break;
    }

    device_ = goal->device_type;

    RCLCPP_INFO(get_logger(), "Received pairing request: %s", msg.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PairingModuleDummy::handle_cancel_(const std::shared_ptr<GoalHandlePairDevice> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PairingModuleDummy::execute_(const std::shared_ptr<GoalHandlePairDevice> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Executing goal");

    auto feedback = std::make_shared<PairDevice::Feedback>();
    auto result = std::make_shared<PairDevice::Result>();

    const auto goal = goal_handle->get_goal();

    for (feedback->progress = 0; (feedback->progress < 1) && rclcpp::ok(); feedback->progress += 0.1)
    {
        // Check if there is a cancel request
        if (goal_handle->is_canceling())
        {
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Goal Canceled");
            return;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(get_logger(), "Publish Feedback");
    }

    if (rclcpp::ok())
    {
        paired_ = (goal->pairing_mode == PairDevice::Goal::PAIR);
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal Succeeded");
    }
}

void PairingModuleDummy::handle_accepted_(const std::shared_ptr<GoalHandlePairDevice> goal_handle)
{
    std::thread{std::bind(&PairingModuleDummy::execute_, this, std::placeholders::_1), goal_handle}.detach();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PairingModuleDummy>());
    rclcpp::shutdown();
    return 0;
}