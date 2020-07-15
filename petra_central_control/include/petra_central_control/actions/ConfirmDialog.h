#pragma once

#include <petra_central_control/default.h>

#include <petra_central_control/actions/base/Action.h>
#include <petra_central_control/actions/types/UserDialog.h>

class ConfirmDialog : public Action
{
public:
    static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("message") }; }

    ConfirmDialog(const std::string &name, const BT::NodeConfiguration &config);

    std::string confirm_msg;

    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    UserDialog confirm_dialog_;
    bool confirmed_ = false;

    void confirm_();
};
