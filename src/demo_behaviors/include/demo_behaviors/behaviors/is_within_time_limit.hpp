#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

#include <optional>
#include <string>


// Mission time-budget guard. Returns SUCCESS while elapsed time is under the
// limit, FAILURE once it is exceeded. Used as the left child of the search
// ReactiveSequence: when it fails, the search branch halts and the mission ends.
//
// Uses the shared node's ROS clock (node->now()) rather than wall time, so the
// budget respects sim time and stays consistent with every other ROS timestamp
// (important for time-varying mission metrics / logging). The start time is
// captured on the first tick and persists across ticks (total elapsed time).
class isWithinTimeLimit : public BT::SyncActionNode
{
    public:
        isWithinTimeLimit(
            const std::string& name,
            const BT::NodeConfig& config,
            rclcpp::Node::SharedPtr node
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        std::optional<rclcpp::Time> start_time_;
};