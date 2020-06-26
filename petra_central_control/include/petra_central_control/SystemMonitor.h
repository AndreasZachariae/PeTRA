#pragma once

#include <petra_central_control/default.h>
#include <petra_central_control/Skill.h>

class SystemMonitor : public Skill
{
private:

public:
    SystemMonitor(std::shared_ptr<rclcpp::Node> node_handle) : Skill(node_handle, "SystemMonitor", true) {}

    void check_system();
protected:
    void spin_();
};
