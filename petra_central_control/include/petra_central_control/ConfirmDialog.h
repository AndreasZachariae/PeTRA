#pragma once

#include <petra_central_control/Skill.h>
#include <petra_central_control/UserDialog.h>

#include <behaviortree_cpp_v3/action_node.h>

//Diese Klasse ist nur als Node im Behavior Tree gedacht, nicht als eigenständiger Skill
class ConfirmDialog : public Skill, public BT::CoroActionNode
{
public:
    ConfirmDialog(std::shared_ptr<rclcpp::Node> node_handle, std::string confirm_msg);
    ConfirmDialog(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<rclcpp::Node> node_handle, std::string confirm_msg);
    
    BT::NodeStatus tick() override;
    void halt() override;

protected:
    void init_() override;
    void spin_() override;

private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    std::string confirm_msg_;
    UserDialog confirm_dialog_;
    bool confirmed_ = false;

    void confirm_();
};
