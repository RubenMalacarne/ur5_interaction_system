#include <rclcpp/rclcpp.hpp>
#include <cr_vision/object_state_manager.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cr::vision::ObjectStateManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}