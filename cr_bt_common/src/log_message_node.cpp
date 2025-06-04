#include "cr_bt_common/log_message_node.hpp"

namespace cr::bt::common
{

    LogMessageNode::LogMessageNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config) {}

    BT::PortsList LogMessageNode::providedPorts()
    {
        return {BT::InputPort<std::string>("message")};
    }

    BT::NodeStatus LogMessageNode::tick()
    {
        auto msg = getInput<std::string>("message");
        if (!msg)
        {
            RCLCPP_ERROR(rclcpp::get_logger("LogMessageNode"), "Missing port [message]");
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("LogMessageNode"), "BT_LOG: %s",
                    msg.value().c_str());
        return BT::NodeStatus::SUCCESS;
    }

} // namespace cr::bt::common