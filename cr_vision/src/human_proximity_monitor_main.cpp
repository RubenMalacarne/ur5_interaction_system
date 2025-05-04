#include <rclcpp/rclcpp.hpp>
#include <cr_vision/human_proximity_monitor.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cr::vision::HumanProximityMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}