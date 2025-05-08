#include "cr_bt_orchestrator/task_planner.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cr::bt_orchestrator::BtOrchestratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
