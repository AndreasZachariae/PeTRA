#pragma once

#include <petra_central_control/Skill.h>

#include <petra_central_control/Sequence.h>

class MissionTest : public Skill
{
public:
    MissionTest(std::shared_ptr<rclcpp::Node> node_handle);

protected:
    void init_() override;
    void spin_() override;

private:
    std::shared_ptr<rclcpp::Node> node_handle_;

    //Typ sollte Skill sein um alles zu ermöglichen
    std::shared_ptr<Sequence> root_skill_;
};
