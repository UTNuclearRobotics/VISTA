#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/action_node.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "uuv_interfaces/action/pose_to_pose.hpp"

#include <optional>

using PoseToPose = uuv_interfaces::action::PoseToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<PoseToPose>;

class DubinsClient: public BT::StatefulActionNode
{
    public:
        DubinsClient(
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
        rclcpp_action::Client<PoseToPose>::SharedPtr dubins_client_;
        
        // Phase 1: future for goal acceptance (gives us a handle)
        std::optional<std::shared_future<GoalHandle::SharedPtr>> goal_handle_future_;
        
        // Phase 2: future for the final result
        std::optional<std::shared_future<GoalHandle::WrappedResult>> result_future_;
        
        // The handle itself once we get it (needed for cancellation in onHalted)
        GoalHandle::SharedPtr goal_handle_;
        
        // TF for converting planning_pose to ned
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        // Client-side timeout tracking
        rclcpp::Time start_time_;
        
        // placeholder; real value set in onStart
        rclcpp::Duration timeout_ {0, 0};










};


