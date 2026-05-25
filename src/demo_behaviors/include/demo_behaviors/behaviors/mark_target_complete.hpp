#pragma once

#include "behaviortree_cpp/action_node.h"

#include <string>


class MarkTargetComplete : public BT::SyncActionNode
{
    public:
        MarkTargetComplete(
            const std::string& name,
            const BT::NodeConfig& config
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
};
