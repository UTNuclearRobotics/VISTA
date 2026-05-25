#include "demo_behaviors/behaviors/dubins_client.hpp"

#include <chrono>

using namespace std::chrono_literals;

DubinsClient::DubinsClient(
    const std::string& name,
    const BT::NodeConfig& config,
    rclcpp::Node::SharedPtr node
) :
    BT::StatefulActionNode(name,config),
    node_(node)
    {
        dubins_client_ = rclcpp_action::create_client<PoseToPose>(node_, "pose_to_pose");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        RCLCPP_INFO(node_->get_logger(), "DubinsClient ready");
    }

BT::PortsList DubinsClient::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("planning_pose"),
        //note the default
        BT::InputPort<double>("timeout_sec", 600.0, "Cancel goal after N seconds (default 10 min)")
    };
}

BT::NodeStatus DubinsClient::onStart(){
    // Clear any state from a previous run so onHalted never cancels a stale handle
    goal_handle_.reset();
    goal_handle_future_ = std::nullopt;
    result_future_ = std::nullopt;

    auto planning_pose = getInput<geometry_msgs::msg::PoseStamped>("planning_pose");
    
     if (!planning_pose.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "planning_pose is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped goal_pose = planning_pose.value();

    try {
        // transform() automatically handles lookupTransform and doTransform internally
        goal_pose = tf_buffer_->transform(goal_pose, "ned");
        RCLCPP_INFO(node_->get_logger(), "Transformed pose to ned frame");
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(node_->get_logger(), "Could not transform to ned: %s", ex.what());
        return BT::NodeStatus::FAILURE;
    }

    auto goal_msg = PoseToPose::Goal();
    goal_msg.goal_pose = goal_pose; 

    RCLCPP_INFO(node_->get_logger(), "Sending Dubins goal");

    // Feedback callback - logs progress as the vehicle drives toward the goal
    auto send_goal_options = rclcpp_action::Client<PoseToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
        [this](GoalHandle::SharedPtr,
               const std::shared_ptr<const PoseToPose::Feedback> fb) {
            RCLCPP_INFO(node_->get_logger(),
                        "Dubins progress: dist=%.2fm yaw_err=%.3frad",
                        fb->distance_remaining,
                        fb->yaw_error_remaining);
        };

    // Load timeout from port (port has default in providedPorts) and record start time
    double timeout_sec = getInput<double>("timeout_sec").value();
    timeout_ = rclcpp::Duration::from_seconds(timeout_sec);
    start_time_ = node_->now();

    // Send the goal - future resolves when server accepts/rejects
    goal_handle_future_ = dubins_client_->async_send_goal(goal_msg, send_goal_options);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DubinsClient::onRunning(){
    // phase 1: wait for goal acceptance

    // we use this state checker to see if we already have a goal handle
    if(!goal_handle_)
    {
        // check optional to see is async send goal has been dispatched properly

        if (!goal_handle_future_.has_value()){
            RCLCPP_ERROR(node_->get_logger(), "onRunning called without an active goal request!");
            return BT::NodeStatus::FAILURE;
        }

        //check if the action server responded
        if (goal_handle_future_->wait_for(0ms) != std::future_status::ready) {
            return BT::NodeStatus::RUNNING;
        }

        // unpack the future to the goal handle pointer
        auto handle =  goal_handle_future_->get();

        // the goal is REJECTED if the handle is a null pointer, so return failure
        if(!handle){
            RCLCPP_ERROR(node_->get_logger(), "Dubins goal was rejected");
            return BT::NodeStatus::FAILURE;
        }

        //else it is accepted (non-null)

        //assign handle to goal handle member variable
        goal_handle_ = handle;
        goal_handle_future_ = std::nullopt; //empties future type member variable wrapped by optional.

        // use goal handle to
        // update member variable with the result future via async get result
        result_future_ = dubins_client_->async_get_result(goal_handle_);
        RCLCPP_INFO(node_->get_logger(), "Dubins goal accepted, awaiting result");
        return BT::NodeStatus::RUNNING;
    }

    //phase 2 is waiting for result (just as long as we do not timeout)

    // client-side timeout: if too much time has elapsed, cancel the goal server-side
    // (the server's cancel_cb gracefully halts the vehicle and restores drift)
    if(node_->now() - start_time_ > timeout_) {
        RCLCPP_WARN(node_->get_logger(),
                    "Dubins timeout after %.1fs, cancelling goal", timeout_.seconds());
        dubins_client_->async_cancel_goal(goal_handle_);
        return BT::NodeStatus::FAILURE;
    }

    //should never happen but good to check optiona;

    if (!result_future_.has_value()){
        RCLCPP_ERROR(node_->get_logger(), "result_future_ not set despite phase 2 entry!");
        return BT::NodeStatus::FAILURE;
    }

    //check if the result future itself is ready (result response)
    if (result_future_->wait_for(0ms) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }

    //unpack result future into wrapped result
    auto wrapped = result_future_->get();
    result_future_ = std::nullopt;  // hygiene: clear once consumed

    // wrapped.code tells us HOW the action ended (SUCCEEDED/CANCELED/ABORTED)
    // wrapped.result is the typed result message (SharedPtr to PoseToPose::Result)
    switch (wrapped.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(),
                        "Dubins succeeded: %s (dist=%.2fm, yaw_err=%.3frad)",
                        wrapped.result->result_message.c_str(),
                        wrapped.result->distance_to_goal,
                        wrapped.result->yaw_error_at_goal);
            return BT::NodeStatus::SUCCESS;

        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Dubins goal was canceled");
            return BT::NodeStatus::FAILURE;

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Dubins goal was aborted by server");
            return BT::NodeStatus::FAILURE;

        default:
            RCLCPP_ERROR(node_->get_logger(), "Dubins result had unknown code");
            return BT::NodeStatus::FAILURE;
    }
}

void DubinsClient::onHalted() {
    // BT was interrupted mid-execution (rare for our tree, but defensive)
    // Cancel the goal so the server actually stops the vehicle
    if (goal_handle_) {
        RCLCPP_INFO(node_->get_logger(), "DubinsClient halted, cancelling goal");
        dubins_client_->async_cancel_goal(goal_handle_);
    }

    // Reset all state so the next onStart starts fresh
    goal_handle_.reset();
    goal_handle_future_ = std::nullopt;
    result_future_ = std::nullopt;

}

