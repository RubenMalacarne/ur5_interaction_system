#include <rclcpp/rclcpp.hpp>
#include <cr_task_orchestration/task_orchestrator.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cr::task_orchestration::TaskOrchestrator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}