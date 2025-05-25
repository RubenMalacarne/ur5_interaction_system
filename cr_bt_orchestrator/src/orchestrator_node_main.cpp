#include "cr_bt_orchestrator/orchestrator_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cr::bt::orchestrator::OrchestratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
