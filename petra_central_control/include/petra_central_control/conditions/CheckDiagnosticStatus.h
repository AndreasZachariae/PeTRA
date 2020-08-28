/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Checks the ROS2-Topic "DiagnosticStatus"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <std_msgs/msg/empty.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <petra_central_control/conditions/base/Condition.h>

class CheckDiagnosticStatus : public Condition
{
public:
    static BT::PortsList providedPorts() { return BT::PortsList(); }

    CheckDiagnosticStatus(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onCheck() override;

private:
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_publisher_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_status_subscription_;

    bool diagnostic_error_ = false;
};
