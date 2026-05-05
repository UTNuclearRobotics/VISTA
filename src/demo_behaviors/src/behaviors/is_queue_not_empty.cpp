#include "demo_behaviors/behaviors/is_queue_not_empty.hpp"


isQueueNotEmpty::isQueueNotEmpty(
      const std::string& name, const BT::NodeConfig& config):
      BT::SyncActionNode(name,config)
{}

BT::PortsList isQueueNotEmpty::providedPorts()
{

 return{ BT::InputPort<std::vector<PoseStamped>>("geoID_read"),
         BT::OutputPort<std::string>("current_ID_write"),
         BT::OutputPort<PoseStamped>("target_pose_write")};
}

BT::NodeStatus isQueueNotEmpty::tick()
{
    BT::Expected<std::vector<PoseStamped>> queue = getInput<std::vector<PoseStamped>>("geoID_read");

    //check if queue is empty or if it is not on the blackboard

    if (!queue || queue.value().empty()){
        return BT::NodeStatus::FAILURE; 
    }

    else{
        // read the first element in the queue
        PoseStamped current_target = queue.value().front();

        // set the string for later
        setOutput("current_ID_write", current_target.header.frame_id);

        //ensure header frame_id is "map" for nbv use
        current_target.header.frame_id = "map";
        setOutput("target_pose_write", current_target);
        return BT::NodeStatus::SUCCESS;

    }



}










