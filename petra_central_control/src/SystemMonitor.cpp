#include <petra_central_control/SystemMonitor.h>


SystemMonitor::SystemMonitor(CentralControlUnit* ccu) 
: Skill(ccu->node_handle, "SystemMonitor", true), ccu_ptr_(ccu)
{
    stop_publisher_ = ccu_ptr_->node_handle->create_publisher<std_msgs::msg::Empty>("Stop", 10);

    battery_state_subscription_ = ccu_ptr_->node_handle->create_subscription<sensor_msgs::msg::BatteryState>("BatteryState", 10, [&](const sensor_msgs::msg::BatteryState::SharedPtr msg) 
    {
        if ((msg->percentage <= 0.2) && !recharge_mission_added_)
        {
            //ccu_ptr_->add_skill(Recharge, MISSION);
            log("[BatteryState] Battery less than 20%, Recharge mission added.", LogLevel::Warn);
            recharge_mission_added_ = true;
        }
        else if(msg->percentage >= 0.2)
        {
            recharge_mission_added_ = false;
        }
    });

    diagnostic_status_subscription_ = ccu_ptr_->node_handle->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>("DiagnosticStatus", 10, [&](const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg) 
    {
        if (msg->level != diagnostic_msgs::msg::DiagnosticStatus::OK)
        {
            log("[DiagnosticStatus] " + msg->name + " ID:" + msg->hardware_id + "Status NOT OK! /Stop published", LogLevel::Error);

            stop_publisher_->publish(std_msgs::msg::Empty());
        }
    });
    
}

void SystemMonitor::spin_()
{
    check_system();
}

void SystemMonitor::check_system()
{
    //log("Background system check...");
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
}