/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PatientMonitoringDummy"
 * Purpose : Provides testing methods and publishes topics.
 *           Has to be extended with full funtionalities.
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <petra_core/default.h>
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

    //only for testing
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr set_condition_subscription_;
};