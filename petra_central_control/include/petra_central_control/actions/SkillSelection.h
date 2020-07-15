#pragma once

#include <petra_central_control/default.h>

#include <petra_central_control/actions/base/Action.h>
#include <petra_central_control/actions/types/UserDialog.h>

class SkillSelection : public Action
{
public:
    static BT::PortsList providedPorts() { return {BT::OutputPort<std::string>("selected_action")}; }

    SkillSelection(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::vector<std::string> default_options_;
    UserDialog selection_dialog_;

    void select_skill_();
    void queue_chosen_skill_(std::string);
};
