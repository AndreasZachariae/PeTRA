#include <petra_central_control/Sequence.h>

Sequence::Sequence(std::shared_ptr<rclcpp::Node> node_handle)
: Skill(node_handle, "Sequence"), node_handle_(node_handle)
{

}

void Sequence::spin_()
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

        case SkillState::failed:
            fail_();
            return;

        default:
            break;
        }
    }

    succeed_();
}
