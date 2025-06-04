#ifndef CR_BT_COMMON__CHECK_BLACKBOARD_KEY_NODE_HPP_
#define CR_BT_COMMON__CHECK_BLACKBOARD_KEY_NODE_HPP_

#include <string>
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>

namespace cr::bt::common
{
    class CheckBlackboardKeyNode : public BT::ConditionNode
    {
    public:
        CheckBlackboardKeyNode(const std::string &name, const BT::NodeConfiguration &config);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        rclcpp::Logger getLogger();
    };

} // namespace cr::bt::common

#endif // CR_BT_COMMON__CHECK_BLACKBOARD_KEY_NODE_HPP_
