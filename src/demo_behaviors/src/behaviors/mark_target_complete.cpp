#include "demo_behaviors/behaviors/mark_target_complete.hpp"


MarkTargetComplete::MarkTargetComplete(
    const std::string& name,
    const BT::NodeConfig& config
) :
    BT::SyncActionNode(name, config)
{}

BT::PortsList MarkTargetComplete::providedPorts() {
    return {
        BT::InputPort<std::string>("current_id_read"),
        BT::OutputPort<std::string>("completed_id_write")
    };
}

BT::NodeStatus MarkTargetComplete::tick() {
    // Read the current target's geometry ID from the blackboard
    auto current_id = getInput<std::string>("current_id_read");
    if (!current_id.has_value()) {
        // Should never happen - isQueueNotEmpty populates current_ID before this runs
        return BT::NodeStatus::FAILURE;
    }

    // Echo the ID to the completed_id_write port
    // DetectAndSortQueue (running in parallel) reads this on next tick and retires the target
    setOutput<std::string>("completed_id_write", current_id.value());

    return BT::NodeStatus::SUCCESS;
}