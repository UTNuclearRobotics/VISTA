#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "uuv_interfaces/srv/generate_helix.hpp"

#include <optional>

using GenerateHelix = uuv_interfaces::srv::GenerateHelix; 

class SampleViewPosesHelix : public BT::StatefulActionNode
{
    public:
        SampleViewPosesHelix(
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
        rclcpp::Client<GenerateHelix>::SharedPtr service_client_;
        std::optional<rclcpp::Client<GenerateHelix>::FutureAndRequestId> future_;
        
};

