#pragma once

#include <petra_central_control/default.h>
#include <petra_central_control/Skill.h>

class SystemMonitor : public Skill
{
private:

public:
    SystemMonitor(std::string name = "SystemMonitor") : Skill(name) {}

    void check_system();
protected:
    void spin_();
};
