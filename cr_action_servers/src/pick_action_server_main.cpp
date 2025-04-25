#include <rclcpp/rclcpp.hpp>
#include <cr_action_servers/pick_action_server.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cr::action_servers::PickActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}