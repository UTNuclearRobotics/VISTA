#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <fstream>
#include <filesystem>
#include <thread>

// Demo behaviors
#include "demo_behaviors/behaviors/detect_and_sort_queue.hpp"
#include "demo_behaviors/behaviors/is_queue_not_empty.hpp"

// Dependency behaviors TODO, needed later


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create Task Planner ROS2 node to execute Behavior Tree
  auto node = std::make_shared<rclcpp::Node>("demo_bt");

  // Spin node in a separate thread so tickWhileRunning() can block the main thread
  // jthread automatically requests stop and joins on destruction (RAII)
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::jthread spin_thread([&executor](std::stop_token stop_token) {
    while (!stop_token.stop_requested()) {
      executor->spin_some();
    }
  });

  // Handles creation of Behavior Tree and registration of Behaviors
  BT::BehaviorTreeFactory factory;

  // Register custom behaviors
  factory.registerNodeType<DetectAndSortQueue>("DetectAndSortQueue", node);
  factory.registerNodeType<isQueueNotEmpty>("isQueueNotEmpty");

  // Register BTs from file
  std::string share_path = ament_index_cpp::get_package_share_directory(
    "demo_behaviors");
  factory.registerBehaviorTreeFromFile(share_path + "/behavior_trees/test_detect.xml");

  // ---- Write Behaviors into TreeNodes Model ---- //
  std::string model_str = BT::writeTreeNodesModelXML(factory);

  // Create path to TreeNodesModel file
  std::filesystem::path package_root = std::filesystem::path(__FILE__).parent_path().parent_path();
  std::filesystem::path models_path = package_root / "config" / "models.xml";

  // Write XML to file
  std::ofstream models_file(models_path);
  if (models_file.is_open()) {
    models_file << model_str;
    models_file.close();
    RCLCPP_INFO(node->get_logger(), "Wrote TreeNodesModel to: %s", models_path.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to write TreeNodesModel to: %s", models_path.c_str());
  }

  // ---- Create and run Behavior Tree ---- //
  auto tree = factory.createTree("TestDetect");

  BT::Groot2Publisher publisher(tree); // Publish to Groot2 to view BT on Port 1667

  // tickWhileRunning() blocks here, repeatedly ticking the tree until it returns
  // SUCCESS or FAILURE. The node spins concurrently on the jthread above, so
  // ROS callbacks (subscribers, services, etc.) are always being serviced while
  // the tree is executing — no manual spin_some() loop needed.
  tree.tickWhileRunning();

  // jthread stop is requested and join happens automatically here on destruction
  executor->cancel();
  rclcpp::shutdown();
  return 0;
}