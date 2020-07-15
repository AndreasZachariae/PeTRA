#pragma once

#include <petra_central_control/default.h>

class Progress : public Component
{
public:
    static const int FAIL_STEP = -1;

    Progress(int steps = 0) : Component("Progress"), steps(steps) {}

    int steps = 0;
    int current_step = 0;
    std::string message = "";

    void next_step(const std::string &message = "");

    void set_step(int step);
    void set_fail();

    std::string to_string();
};