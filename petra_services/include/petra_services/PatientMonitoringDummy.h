/*****************************************************
 *                    ROS2 Node
 *****************************************************/
#pragma once

#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <petra_core/msg/patient_condition.hpp>
#include <petra_core/msg/emergency_stop.hpp>

class PatientMonitoringDummy : public rclcpp::Node
{
public:
    PatientMonitoringDummy();

private:
    rclcpp::Publisher<petra_core::msg::PatientCondition>::SharedPtr condition_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_publisher_;
    rclcpp::Publisher<petra_core::msg::EmergencyStop>::SharedPtr emergency_publisher_;

    //only for debugging
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr set_condition_subscription_;
    void set_condition_callback_(const std_msgs::msg::String::SharedPtr msg);
};