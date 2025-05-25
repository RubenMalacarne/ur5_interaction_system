#ifndef CR_BT_COMMON__LOG_MESSAGE_NODE_HPP_
#define CR_BT_COMMON__LOG_MESSAGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/behavior_tree.h>

namespace cr::bt::common
{
    class LogMessageNode : public BT::SyncActionNode
    {
    public:
        LogMessageNode(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
    };
} // namespace cr::bt::common

#endif