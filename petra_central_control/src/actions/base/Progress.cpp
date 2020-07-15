#include <petra_central_control/actions/base/Progress.h>

void Progress::next_step(const std::string &message)
{
    this->message = message;

    ++current_step;

    log(to_string());
}

void Progress::set_step(int step)
{
    if (step > 0 && step <= steps)
    {
        current_step = step;
    }
    else
    {
        log("step " + std::to_string(current_step) + " is out of range 0 to " + std::to_string(step));
    }
}

void Progress::set_fail()
{
    current_step = FAIL_STEP;
}

std::string Progress::to_string()
{
    return "Step " + std::to_string(current_step) + "/" + std::to_string(steps) + ": " + message;
}