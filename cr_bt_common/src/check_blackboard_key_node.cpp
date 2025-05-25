#include "cr_bt_common/check_blackboard_key_node.hpp"

namespace cr::bt::common
{
    CheckBlackboardKeyNode::CheckBlackboardKeyNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ConditionNode(name, config)
    {
    }

    BT::PortsList CheckBlackboardKeyNode::providedPorts()
    {
        return {
            BT::InputPort<std::string>("key", "The blackboard key to check for existence.")};
    }

    BT::NodeStatus CheckBlackboardKeyNode::tick()
    {
        BT::Expected<std::string> key_to_check_expected = getInput<std::string>("key");

        if (!key_to_check_expected)
        {
            RCLCPP_ERROR(
                getLogger(),
                "CheckBlackboardKeyNode (%s): missing required input [key]: %s",
                this->name().c_str(), key_to_check_expected.error().c_str());
            return BT::NodeStatus::FAILURE;
        }

        const std::string &key_to_check = key_to_check_expected.value();
        const auto blackboard = config().blackboard;

        if (!blackboard)
        {
            RCLCPP_ERROR(
                getLogger(),
                "CheckBlackboardKeyNode (%s): Blackboard is null.",
                this->name().c_str());
            return BT::NodeStatus::FAILURE;
        }

        if (blackboard->getEntry(key_to_check) != nullptr)
        {
            RCLCPP_DEBUG(
                getLogger(),
                "CheckBlackboardKeyNode (%s): Key '%s' found in blackboard.",
                this->name().c_str(), key_to_check.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_DEBUG(
                getLogger(),
                "CheckBlackboardKeyNode (%s): Key '%s' NOT found in blackboard.",
                this->name().c_str(), key_to_check.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    rclcpp::Logger CheckBlackboardKeyNode::getLogger()
    {
        rclcpp::Node::SharedPtr node = nullptr;
        if (config().blackboard->get("ros_node", node) && node)
        {
            return node->get_logger().get_child("CheckBlackboardKeyNode");
        }
        return rclcpp::get_logger("CheckBlackboardKeyNode");
    }

} // namespace cr::bt::common