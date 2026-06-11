#include "demo_behaviors/behaviors/path_length_client.hpp"

#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

PathLengthClient::PathLengthClient(
    const std::string& name,
    const BT::NodeConfig& config,
    rclcpp::Node::SharedPtr node
    ):
        BT::StatefulActionNode(name, config),
        node_(node)
        {
            service_client_ = node_->create_client<DubinsLengths>("compute_path_lengths");
            RCLCPP_INFO(node_->get_logger(), "PathLengthClient client ready");
        }

BT::PortsList PathLengthClient::providedPorts(){
    return {
                BT::InputPort<geometry_msgs::msg::PoseArray>("view_poses"),
                BT::InputPort<std::string>("robot_frame_id"),
                BT::InputPort<std::string>("view_frame_id"),
                BT::InputPort<double>("turn_radius"),
                BT::InputPort<double>("max_pitch_deg"),
                BT::OutputPort<std::vector<double>>("path_costs")};
}

BT::NodeStatus PathLengthClient::onStart() {

    auto view_poses = getInput<geometry_msgs::msg::PoseArray>("view_poses");
    if (!view_poses.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "view_poses is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto robot_frame_id = getInput<std::string>("robot_frame_id");
    if (!robot_frame_id.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "robot_frame_id is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto view_frame_id = getInput<std::string>("view_frame_id");
    if (!view_frame_id.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "view_frame_id is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto turn_radius = getInput<double>("turn_radius");
    if (!turn_radius.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "turn_radius is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto max_pitch_deg = getInput<double>("max_pitch_deg");
    if (!max_pitch_deg.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "max_pitch_deg is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    // Empty candidate list: nothing to cost, succeed with an empty array.
    if (view_poses->poses.empty()) {
        setOutput<std::vector<double>>("path_costs", {});
        return BT::NodeStatus::SUCCESS;
    }

    auto req = std::make_shared<DubinsLengths::Request>();
    req->nbv_poses = view_poses.value();
    req->robot_frame_id = robot_frame_id.value();
    req->view_frame_id = view_frame_id.value();
    req->turn_radius = turn_radius.value();
    req->max_pitch_deg = max_pitch_deg.value();

    future_ = service_client_->async_send_request(req);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PathLengthClient::onRunning() {

    if (!future_.has_value()){
        RCLCPP_ERROR(node_->get_logger(), "onRunning called without an active request!");
        return BT::NodeStatus::FAILURE;
    }

    if (future_->wait_for(0ms) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }

    auto resp = future_->get();

    if (!resp) {
        RCLCPP_ERROR(node_->get_logger(), "Path length service returned null response");
        future_ = std::nullopt;
        return BT::NodeStatus::FAILURE;
    }

    setOutput<std::vector<double>>("path_costs", resp->path_lengths);

    std::stringstream ss;
    ss << "Computed " << resp->path_lengths.size() << " path costs: [";
    for (std::size_t i = 0; i < resp->path_lengths.size(); ++i) {
        ss << resp->path_lengths[i];
        if (i + 1 < resp->path_lengths.size()) ss << ", ";
    }
    ss << "]";
    RCLCPP_DEBUG_STREAM(node_->get_logger(), ss.str());

    future_ = std::nullopt;
    return BT::NodeStatus::SUCCESS;
}

void PathLengthClient::onHalted() {
    if (future_.has_value()) {
        service_client_->remove_pending_request(future_.value());
    }
    future_ = std::nullopt;
}