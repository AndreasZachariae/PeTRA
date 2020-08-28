/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACentralControl"
 * Purpose : Progress for internal state of RosActions
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <petra_central_control/default.h>

/**
 * Provides a progress value and information logging 
 * for the internal state-machine of each RosAction.
 */
class Progress : public Component
{
public:
    /// Used to return FAILURE of a BT-Node
    static const int FAIL_STEP = -1;

    Progress(int steps = 0) : Component("Progress"), steps(steps) {}

    /// Has to be initialized with the correct number of steps
    int steps = 0;
    int current_step = 0;
    std::string message = "";

    /// Increments current_step and logs the given message.
    void next_step(const std::string &message = "");

    /// Sets current_step to given number and logs the message.
    void set_step(int step, const std::string &message = "");

    /// Sets current_step to FAIL_STEP and logs error message.
    void set_fail(const std::string &message = "");

    /// Used to log the current progress to the console.
    std::string to_string();
};