#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <mutex>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using GoalList = nav_msgs::msg::Path;
using PoseStamped = geometry_msgs::msg::PoseStamped; 


class DetectAndSortQueue : public BT::SyncActionNode
{
    public:

    DetectAndSortQueue(const std::string& name, const BT::NodeConfig& config, 
        rclcpp::Node::SharedPtr node);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override; 


    private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<GoalList>::SharedPtr subscription_; 
    std::vector<PoseStamped> queue_;
    std::unordered_map<std::string,PoseStamped> visited_hashmap_;
    
    //mailbox style, will get reset after being read in tick()
    GoalList::SharedPtr new_msg_; 

    // prevent callback and tick data races for updating new msg
    std::mutex mtx_; 

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
}; 


