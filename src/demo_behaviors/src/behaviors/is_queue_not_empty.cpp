#include "demo_behaviors/behaviors/is_queue_not_empty.hpp"


isQueueNotEmpty::isQueueNotEmpty(
      const std::string& name, const BT::NodeConfig& config):
      BT::SyncActionNode(name,config)
{}

BT::PortsList isQueueNotEmpty::providedPorts()
{

 return{ BT::InputPort<std::vector<PoseStamped>>("geoID_write"),
         BT::OutputPort<PoseStamped>("current_ID")};
}

BT::NodeStatus isQueueNotEmpty::tick()
{
    BT::Expected<std::vector<PoseStamped>> queue = getInput<std::vector<PoseStamped>>("geoID_write");

    //check if queue is empty or if it is not on the blackboard

    if (!queue || queue.value().empty()){
        return BT::NodeStatus::FAILURE; 
    }

    else{
        // read the first element in the queue
        PoseStamped current_target = queue.value().front();
        setOutput("current_ID", current_target);

        return BT::NodeStatus::SUCCESS;

    }



}










