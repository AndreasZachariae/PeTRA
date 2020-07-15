#include <petra_central_control/FreeWalking.h>

#include <petra_central_control/BaseMovement.h>
#include <petra_central_control/ConfirmDialog.h>

FreeWalking::FreeWalking(CentralControlUnit* ccu) : Skill(ccu->node_handle, "FreeWalking"), ccu_ptr_(ccu)
{
    condition_subscription_ = ccu_ptr_->node_handle->create_subscription<petra_core::msg::PatientCondition>("PatientCondition", 10, [&](const petra_core::msg::PatientCondition::SharedPtr msg) 
    {
        patient_condition_ = msg->condition;
    });
}

BT::NodeStatus FreeWalking::check_condition()
{
    if (patient_condition_ == petra_core::msg::PatientCondition::GOOD)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

void FreeWalking::init_()
{
    BT::BehaviorTreeFactory factory;

    factory.registerSimpleCondition("CheckCondition", std::bind(&FreeWalking::check_condition, this));

    factory.registerBuilder<BaseMovement>( "Movement1", [&](const std::string& name, const BT::NodeConfiguration& config)
    {
        auto skill_shared = std::make_shared<BaseMovement>(name, config, ccu_ptr_->node_handle);

        skill_shared->set_goal(room1_);

        ccu_ptr_->add_skill(skill_shared);
        
        return skill_shared;
    });

    factory.registerBuilder<BaseMovement>( "Movement2", [&](const std::string& name, const BT::NodeConfiguration& config)
    {
        auto skill_shared = std::make_shared<BaseMovement>(name, config, ccu_ptr_->node_handle);

        skill_shared->set_goal(room2_);

        ccu_ptr_->add_skill(skill_shared);
        
        return skill_shared;
    });

    factory.registerBuilder<BaseMovement>( "Movement3", [&](const std::string& name, const BT::NodeConfiguration& config)
    {
        auto skill_shared = std::make_shared<BaseMovement>(name, config, ccu_ptr_->node_handle);

        skill_shared->set_goal(waiting_area_);

        ccu_ptr_->add_skill(skill_shared);
        
        return skill_shared;
    });

    factory.registerBuilder<ConfirmDialog>( "Confirm", [&](const std::string& name, const BT::NodeConfiguration& config)
    {
        auto skill_shared = std::make_shared<ConfirmDialog>(name, config, ccu_ptr_->node_handle, "[y]continue mission [n]cancel");

        ccu_ptr_->add_skill(skill_shared);
        
        return skill_shared;
    });

    behavior_tree_ = factory.createTreeFromFile("/home/andreas/petra_ws/src/petra_central_control/behaviors/free_walking.xml");


/*
    for (auto &node : behavior_tree_.nodes)
    {
        if (node.get()->name() == "Movement1")
        {
            std::shared_ptr<BaseMovement> skill_shared(dynamic_cast<BaseMovement *>(node.get()));
            ccu_ptr_->add_skill(skill_shared);
            skill_shared->set_goal(room1_);

            log(node.get()->name() + " cast and goal set");
        }
        else if (node.get()->name() == "Movement2")
        {
            std::shared_ptr<BaseMovement> skill_shared(dynamic_cast<BaseMovement *>(node.get()));
            ccu_ptr_->add_skill(skill_shared);
            skill_shared->set_goal(room2_);

            log(node.get()->name() + " cast and goal set");
        }
        else if (node.get()->name() == "Movement3")
        {
            std::shared_ptr<BaseMovement> skill_shared(dynamic_cast<BaseMovement *>(node.get()));
            ccu_ptr_->add_skill(skill_shared);
            skill_shared->set_goal(waiting_area_);

            log(node.get()->name() + " cast and goal set");
        }
        else if (node.get()->name() == "Confirm")
        {
            std::shared_ptr<ConfirmDialog> skill_shared(dynamic_cast<ConfirmDialog *>(node.get()));
            ccu_ptr_->add_skill(skill_shared);

            log(node.get()->name() + " added");
        }
    }
*/
}

void FreeWalking::spin_()
{
    BT::NodeStatus status = behavior_tree_.tickRoot();

    if((status == BT::NodeStatus::SUCCESS))
    {
        log("[BehaviorTree] Tree finished with SUCCESS");
        succeed_();
    }

    if((status == BT::NodeStatus::FAILURE))
    {
        log("[BehaviorTree] Tree finished with FAILURE");

        //stop() instead of fail_() to remove queued skills
        stop();
    }
}

void FreeWalking::stop_()
{
    ccu_ptr_->pop_all_skills();
}