#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

#include "geometry_msgs/msg/pose_array.hpp"

#include "uuv_interfaces/srv/dubins_lengths.hpp"

#include <optional>

using DubinsLengths = uuv_interfaces::srv::DubinsLengths;

class PathLengthClient : public BT::StatefulActionNode
{
    public:
        PathLengthClient(
            const std::string& name,
            const BT::NodeConfig& config,
            rclcpp::Node::SharedPtr node
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<DubinsLengths>::SharedPtr service_client_;
        std::optional<rclcpp::Client<DubinsLengths>::FutureAndRequestId> future_;

};