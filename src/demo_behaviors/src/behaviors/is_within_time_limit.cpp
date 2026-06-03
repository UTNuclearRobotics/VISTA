#include "demo_behaviors/behaviors/is_within_time_limit.hpp"


isWithinTimeLimit::isWithinTimeLimit(
    const std::string& name,
    const BT::NodeConfig& config,
    rclcpp::Node::SharedPtr node
) :
    BT::SyncActionNode(name, config),
    node_(node)
{}

BT::PortsList isWithinTimeLimit::providedPorts() {
    return { BT::InputPort<double>("hours", "Mission time budget in hours") };
}

BT::NodeStatus isWithinTimeLimit::tick() {
    auto hours = getInput<double>("hours");
    if (!hours.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "hours is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    // capture start on first tick; persists across subsequent ticks
    if (!start_time_.has_value()) {
        start_time_ = node_->now();
    }

    const double elapsed_sec = (node_->now() - start_time_.value()).seconds();
    const double limit_sec = hours.value() * 3600.0;

    if (elapsed_sec < limit_sec) {
        return BT::NodeStatus::SUCCESS;   // still within budget
    }

    RCLCPP_WARN(node_->get_logger(),
                "Mission time budget of %.2f h exceeded (elapsed %.1f s); ending search",
                hours.value(), elapsed_sec);
    return BT::NodeStatus::FAILURE;
}
