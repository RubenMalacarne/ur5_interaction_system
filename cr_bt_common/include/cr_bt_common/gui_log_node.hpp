#ifndef CR_BT_COMMON__GUI_LOG_NODE_HPP_
#define CR_BT_COMMON__GUI_LOG_NODE_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace cr::bt::common
{
    class GuiLog : public BT::SyncActionNode
    {
    public:
        GuiLog(const std::string& name, const BT::NodeConfig& config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    };
}

#endif