#include "demo_behaviors/behaviors/sample_view_poses_helix.hpp"

#include <chrono>

using namespace std::chrono_literals;

SampleViewPosesHelix::SampleViewPosesHelix(
    const std::string& name,
    const BT::NodeConfig& config,
    rclcpp::Node::SharedPtr node
    ):
        BT::StatefulActionNode(name,config),
        node_(node)
        {
            service_client_ = node_->create_client<GenerateHelix>("generate_helix");
            RCLCPP_INFO(node_->get_logger(), "SampleViewPosesHelix client ready");
        }

 BT::PortsList SampleViewPosesHelix::providedPorts(){
    return {    
                BT::InputPort<double>("min_radius"),
                BT::InputPort<double>("radius_multiplier"),
                BT::InputPort<int>("num_shells"),
                BT::InputPort<double>("helix_height"),
                BT::InputPort<double>("clearance_height"),
                BT::InputPort<double>("psi_max_deg"),
                BT::InputPort<double>("delta_theta"),
                BT::InputPort<double>("mount_angle_deg"),
                BT::OutputPort<geometry_msgs::msg::PoseArray>("view_poses")};
 }

 BT::NodeStatus SampleViewPosesHelix::onStart() {

    // Validate all input ports
    auto min_radius = getInput<double>("min_radius");
    if (!min_radius.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "min_radius is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto radius_multiplier = getInput<double>("radius_multiplier");
    if (!radius_multiplier.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "radius_multiplier is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto num_shells = getInput<int>("num_shells");
    if (!num_shells.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "num_shells is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto helix_height = getInput<double>("helix_height");
    if (!helix_height.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "helix_height is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto clearance_height = getInput<double>("clearance_height");
    if (!clearance_height.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "clearance_height is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto psi_max_deg = getInput<double>("psi_max_deg");
    if (!psi_max_deg.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "psi_max_deg is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto delta_theta = getInput<double>("delta_theta");
    if (!delta_theta.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "delta_theta is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }

    auto mount_angle_deg = getInput<double>("mount_angle_deg");
    if (!mount_angle_deg.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "mount_angle_deg is not set on blackboard!");
        return BT::NodeStatus::FAILURE;
    }
    // validation block

    //request

    auto req = std::make_shared<GenerateHelix::Request>();
    req->r_min = min_radius.value();
    req->radius_multiplier = radius_multiplier.value();
    req->num_shells = num_shells.value();
    req->helix_height = helix_height.value();
    req->clearance = clearance_height.value();
    req->psi_max_deg = psi_max_deg.value();
    req->delta_theta_deg = delta_theta.value();
    req->mount_angle_deg = mount_angle_deg.value();

    future_ = service_client_->async_send_request(req);

    return BT::NodeStatus::RUNNING;
}

 BT::NodeStatus SampleViewPosesHelix::onRunning() {

    if (!future_.has_value()){
        RCLCPP_ERROR(node_->get_logger(), "onRunning called without an active request!");
        return BT::NodeStatus::FAILURE;
    }

    if (future_->wait_for(0ms) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }
    
    auto resp = future_->get(); 

    // Check we got something back
    if (!resp) {
        RCLCPP_ERROR(node_->get_logger(), "Helix service returned null response");
        future_ = std::nullopt;
        return BT::NodeStatus::FAILURE;
    }

    // Check poses array is not empty
    if (resp->poses.poses.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Helix service returned no poses");
        future_ = std::nullopt;
        return BT::NodeStatus::FAILURE;
    }


    setOutput<geometry_msgs::msg::PoseArray>("view_poses", resp->poses);
    RCLCPP_INFO(node_->get_logger(), "Generated %zu helix poses", resp->poses.poses.size());

    future_ = std::nullopt;
    return BT::NodeStatus::SUCCESS;
}


void SampleViewPosesHelix::onHalted() {
    if (future_.has_value()) {
        service_client_->remove_pending_request(future_.value());
    }
    future_ = std::nullopt;
}















 

 

