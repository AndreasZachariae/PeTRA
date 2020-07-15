#pragma once

#include <petra_central_control/CentralControlUnit.h>
#include <petra_central_control/UserDialog.h>

#include <behaviortree_cpp_v3/action_node.h>

class SkillSelection : public Skill, public BT::AsyncActionNode
{
public:
    SkillSelection(const std::string& name, const BT::NodeConfiguration& config, CentralControlUnit* ccu);
    SkillSelection(CentralControlUnit* ccu);
    
    BT::NodeStatus tick() override;
    void halt() override;

protected:
    void init_() override;
    void spin_() override;

private:
    std::vector<std::string> default_options_;
    CentralControlUnit* ccu_ptr_ = nullptr;
    UserDialog selection_dialog_;

    void select_skill_();
    void queue_chosen_skill_(std::string);

    std::atomic_bool halt_requested_;
};
