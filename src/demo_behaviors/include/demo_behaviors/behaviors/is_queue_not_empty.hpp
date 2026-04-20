#pragma once

#include "behaviortree_cpp/action_node.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>




class isQueueNotEmpty : public BT::SyncActionNode
{
    public:

    using PoseStamped = geometry_msgs::msg::PoseStamped; 


    isQueueNotEmpty(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override; 
    
 

}; 


