/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Checks the ROS2-Topic "Stop"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

#include <std_msgs/msg/empty.hpp>

#include <petra_central_control/conditions/base/Condition.h>

class CheckStop : public Condition
{
public:
    static BT::PortsList providedPorts() { return BT::PortsList(); }

    CheckStop(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onCheck() override;

private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;

    bool stop_recieved_ = false;
};
