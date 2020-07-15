#include <petra_central_control/Fallback.h>

Fallback::Fallback(std::shared_ptr<rclcpp::Node> node_handle)
: Skill(node_handle, "Fallback"), node_handle_(node_handle)
{

}

void Fallback::spin_()
{
    for (auto &&child : child_skills)
    {
        switch (child->get_state())
        {
        case SkillState::uninitialized:
        case SkillState::initialized:
            child->start();

        case SkillState::active:
            child->spin();
            return;

        case SkillState::succeeded:
            succeed_();
            return;

        default:
            break;
        }
    }

    fail_();
}
