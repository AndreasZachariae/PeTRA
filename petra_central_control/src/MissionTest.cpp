#include <petra_central_control/MissionTest.h>

#include <petra_central_control/BaseMovement.h>
#include <petra_central_control/ConfirmDialog.h>
#include <petra_central_control/Sequence.h>

MissionTest::MissionTest(std::shared_ptr<rclcpp::Node> node_handle) : Skill(node_handle, "MissionTest"), node_handle_(node_handle)
{

}

void MissionTest::init_()
{
    root_skill_ = std::make_shared<Sequence>(node_handle_);

    root_skill_->child_skills.push_back(std::make_shared<BaseMovement>(node_handle_));
    root_skill_->child_skills.push_back(std::make_shared<ConfirmDialog>(node_handle_, "[y]continue mission [n]cancel"));
    root_skill_->child_skills.push_back(std::make_shared<BaseMovement>(node_handle_));
    root_skill_->child_skills.push_back(std::make_shared<ConfirmDialog>(node_handle_, "[y]continue mission [n]cancel"));
    root_skill_->child_skills.push_back(std::make_shared<BaseMovement>(node_handle_));
    
}

void MissionTest::spin_()
{
    switch (root_skill_->get_state())
    {
    case SkillState::uninitialized:
    case SkillState::initialized:
        root_skill_->start();
        break;

    case SkillState::succeeded:
        succeed_();
        break;

    case SkillState::failed:
        fail_();
        break;

    case SkillState::active:
        root_skill_->spin();
        break;
    
    default:
        break;
    }
}