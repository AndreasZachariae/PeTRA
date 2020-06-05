#pragma once

#include <petra_central_control/default.h>

class Progress
{
private:
public:
    Progress(int steps = 0) : steps(steps) {}
    ~Progress() {}

    int steps = 0;
    int current_step = 0;
    std::string msg = "";

    void next_step() { ++current_step; }

    std::string to_string() { return "Step " + std::to_string(current_step) + "/" + std::to_string(steps) + ": " + msg; }
};