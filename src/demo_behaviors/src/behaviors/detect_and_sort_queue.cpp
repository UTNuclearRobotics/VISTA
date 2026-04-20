#include "demo_behaviors/behaviors/detect_and_sort_queue.hpp"


DetectAndSortQueue::DetectAndSortQueue(
    const std::string& name, const BT::NodeConfig& config, 
    rclcpp::Node::SharedPtr node
    ):
     BT::SyncActionNode(name,config),
     node_(node)
     {
        //create a subscriber topic_callback lambda
        //lock guard for data race, pass in msg as pointer to prevent copy
        auto topic_cb = [this](const GoalList::SharedPtr msg){
            std::lock_guard<std::mutex> lock(mtx_);
            new_msg_ = msg;

        };

        subscription_ = node_->create_subscription<GoalList>(
        "/detected_boxes",10, topic_cb);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
     }

BT::PortsList DetectAndSortQueue::providedPorts()
{
    return { BT::OutputPort<std::vector<PoseStamped>>("geoID_write"),
            //default empty entry.
             BT::InputPort<std::string>("completed_ID_read")};
}

BT::NodeStatus DetectAndSortQueue::tick()
{
    //check input port to update the visited_hashmao_

    BT::Expected<std::string> completedID_msg = getInput<std::string>("completed_ID_read");

    if(completedID_msg)
    {
        std::string c_id = completedID_msg.value();

        auto it = std::find_if(queue_.begin(), queue_.end(), [&c_id](const PoseStamped& p) 
            { return p.header.frame_id == c_id;});
        
        // assign contents without copy, then erase destroys element
        visited_hashmap_[c_id] = std::move(*it); 
        queue_.erase(it);
    }


    // let's be safer and copy the pointer
    //  we can create a scope for the lock guard to copy the pointer
    // then clear the new_msg_ pointer (to prevent us from reusing the same
    // populated msg. this allows for easy !msg checks)


    
    GoalList::SharedPtr msg;
    // scope for lock guard
    {
        std::lock_guard<std::mutex> lock(mtx_);
        msg = new_msg_;
        new_msg_.reset();

    } // lock released

    // log queue contents every tick (sorted order, closest first)
    std::string ids;
    for (const auto& p : queue_) {
        ids += p.header.frame_id + " ";
    }
    RCLCPP_INFO(node_->get_logger(), "queue size=%zu msg=%s ids=[ %s]",
        queue_.size(), msg ? "received" : "null", ids.c_str());

    // now use msg

    if(!msg){

        return BT::NodeStatus::SUCCESS;

    }
    
    else{
        for(const auto& pose_stamped: msg->poses){
            std::string curr_id = pose_stamped.header.frame_id;
            //check if this id has already been visited (visited_hashmap)
            // or if it is already enqueued

            // use a lambda to check the queue based on id
            auto it = std::find_if(queue_.begin(), queue_.end(), [&curr_id](const PoseStamped& p)
            { return p.header.frame_id == curr_id;});

            if(visited_hashmap_.count(curr_id) or (it != queue_.end())){
                continue;
            }

            else{
                queue_.push_back(pose_stamped);
            }
        }
    }

    // re-sort every tick so the queue reflects current robot position,
    // since the robot moves relative to NED between message arrivals
    if(!queue_.empty()){
        geometry_msgs::msg::TransformStamped robot_tf;
        try {
            // robot pose relative to map
            robot_tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
            return BT::NodeStatus::SUCCESS;  // skip this tick
        }

        double rx = robot_tf.transform.translation.x;
        double ry = robot_tf.transform.translation.y;
        double rz = robot_tf.transform.translation.z;

        // this chain gives the displacement from the target to the map and then map to robot
        // resultant displacement is a vector from the target to the robot
        // just compute the distance squared
        std::sort(queue_.begin(), queue_.end(), [rx,ry,rz](const PoseStamped& a, const PoseStamped& b)
        {
            double a_x = rx - a.pose.position.x;
            double a_y = ry - a.pose.position.y;
            double a_z = rz - a.pose.position.z;

            double dist_a = (a_x*a_x) + (a_y*a_y) + (a_z*a_z);

            double b_x = rx - b.pose.position.x;
            double b_y = ry - b.pose.position.y;
            double b_z = rz - b.pose.position.z;

            double dist_b = (b_x*b_x) + (b_y*b_y) + (b_z*b_z);

            return dist_a < dist_b;
        });

        setOutput("geoID_write", queue_);
    }

    return BT::NodeStatus::SUCCESS;


}
