#include "cr_bt_orchestrator/custom_bt_nodes.hpp"

namespace cr {
namespace bt_nodes {

BT::NodeStatus GetObjectInfo::tick() {
  auto object_id = getInput<std::string>("object_id");
  if (!object_id) {
    RCLCPP_ERROR(rclcpp::get_logger("GetObjectInfoNode"),
                 "Missing port [object_id]");
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("GetObjectInfoNode"),
              "Simulating GetObjectInfo for: %s", object_id.value().c_str());

  // Simuliamo di aver trovato l'oggetto
  cr_interfaces::msg::ObjectInfo object_info;
  object_info.id = 0;
  object_info.center.x = 0.939;
  object_info.center.y = 0.625;
  object_info.center.z = 0.988;
  object_info.size.x = 0.05;
  object_info.size.y = 0.05;
  object_info.size.z = 0.05;

  setOutput("object_info", object_info);

  RCLCPP_INFO(rclcpp::get_logger("GetObjectInfoNode"),
              "Object %s found at [%.2f, %.2f, %.2f]",
              object_id.value().c_str(), object_info.center.x,
              object_info.center.y, object_info.center.z);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ExecutePick::tick() {
  auto object_info = getInput<cr_interfaces::msg::ObjectInfo>("object_info");

  if (!object_info) {
    RCLCPP_ERROR(rclcpp::get_logger("ExecutePickNode"),
                 "Missing required input ports for ExecutePick");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ExecutePickNode"),
              "Simulating ExecutePick for object");

  RCLCPP_INFO(rclcpp::get_logger("ExecutePickNode"),
              "Pick successful (simulated).");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ExecutePlace::tick() {
  auto object_info = getInput<cr_interfaces::msg::ObjectInfo>("object_info");

  if (!object_info) {
    RCLCPP_ERROR(rclcpp::get_logger("ExecutePlaceNode"),
                 "Missing required input ports for ExecutePlace");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ExecutePlaceNode"),
              "Simulating ExecutePlace for object");

  RCLCPP_INFO(rclcpp::get_logger("ExecutePlaceNode"),
              "Place successful (simulated).");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nodes
}  // namespace cr
