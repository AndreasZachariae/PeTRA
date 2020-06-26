#pragma once

#include <petra_central_control/default.h>
#include <petra_central_control/Skill.h>
#include <petra_central_control/CentralControlUnit.h>
#include <petra_central_control/UserDialog.h>

class SkillSelection : public Skill
{
private:
    std::vector<std::string> default_options_;
    CentralControlUnit* ccu_ptr_;
    UserDialog selection_dialog_;

    void select_skill_();
    void queue_chosen_skill_(std::string);
public:
    SkillSelection(CentralControlUnit* ccu);

protected:
    void init_() override;
    void spin_() override;
};
