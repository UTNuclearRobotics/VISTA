#include "demo_behaviors/behaviors/get_next_search_pose.hpp"

#include <chrono>

using namespace std::chrono_literals;

GetNextSearchPose::GetNextSearchPose(
    const std::string& name,
    const BT::NodeConfig& config,
    rclcpp::Node::SharedPtr node
    ):
        BT::StatefulActionNode(name, config),
        node_(node)
        {
            service_client_ = node_->create_client<BayesianSearch>("/bayesian_search_server/get_next_search_pose");
            RCLCPP_INFO(node_->get_logger(), "GetNextSearchPose client ready");
        }

BT::PortsList GetNextSearchPose::providedPorts() {
    return {
        // completed-target map (id -> map-frame pose) from DetectAndSortQueue
        BT::InputPort<std::unordered_map<std::string, PoseStamped>>("visited_ID_read"),
        // next waypoint to navigate to (NED, FLS-offset), consumed by DubinsClient
        BT::OutputPort<PoseStamped>("planning_pose_write")
    };
}

BT::NodeStatus GetNextSearchPose::onStart() {
    auto req = std::make_shared<BayesianSearch::Request>();

    // The visited map may be absent/empty before any target completes -- that's
    // fine: the server just applies no bumps and returns the current argmax.
    auto visited = getInput<std::unordered_map<std::string, PoseStamped>>("visited_ID_read");
    if (visited.has_value()) {
        // flatten the map values into the request array; each pose carries its
        // geometry ID in header.frame_id (used by the server for delta tracking).
        // key is ignored -- it duplicates pose.header.frame_id
        for (const auto& [_, pose] : visited.value()) {
            req->visited_targets.push_back(pose);
        }
    }

    future_ = service_client_->async_send_request(req);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetNextSearchPose::onRunning() {
    if (!future_.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "onRunning called without an active request!");
        return BT::NodeStatus::FAILURE;
    }

    if (future_->wait_for(0ms) != std::future_status::ready) {
        return BT::NodeStatus::RUNNING;
    }

    auto resp = future_->get();
    future_ = std::nullopt;

    if (!resp) {
        RCLCPP_ERROR(node_->get_logger(), "BayesianSearch service returned null response");
        return BT::NodeStatus::FAILURE;
    }

    setOutput<PoseStamped>("planning_pose_write", resp->next_pose);
    RCLCPP_INFO(node_->get_logger(),
                "Next search pose: ned (%.2f, %.2f) utility=%.3f exhausted=%d",
                resp->next_pose.pose.position.x,
                resp->next_pose.pose.position.y,
                resp->cell_utility,
                resp->search_exhausted);

    return BT::NodeStatus::SUCCESS;
}

void GetNextSearchPose::onHalted() {
    if (future_.has_value()) {
        service_client_->remove_pending_request(future_.value());
    }
    future_ = std::nullopt;
}
