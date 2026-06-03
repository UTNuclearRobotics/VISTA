#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

#include "geometry_msgs/msg/pose_stamped.hpp"


#include "uuv_interfaces/srv/bayesian_search.hpp"

#include <optional>
#include <unordered_map> //used for the input port hashmap

using BayesianSearch = uuv_interfaces::srv::BayesianSearch;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class GetNextSearchPose : public BT::StatefulActionNode
{
    public:
        GetNextSearchPose(
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
        rclcpp::Client<BayesianSearch>::SharedPtr service_client_;
        std::optional<rclcpp::Client<BayesianSearch>::FutureAndRequestId> future_;

};