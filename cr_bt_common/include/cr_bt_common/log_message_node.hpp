/**
 * @file log_message_node.hpp
 * @brief Behavior Tree node that logs a custom message to the console.
 *
 * This node is useful for debugging or to have structured logs during BT execution.
 */

#ifndef CR_BT_COMMON__LOG_MESSAGE_NODE_HPP_
#define CR_BT_COMMON__LOG_MESSAGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/behavior_tree.h>

namespace cr::bt::common
{

    /**
     * @class LogMessageNode
     * @brief BT node that logs an input string using ROS logging.
     *
     * Logs a message retrieved from the blackboard input port "message".
     */
    class LogMessageNode : public BT::SyncActionNode
    {
    public:
        /**
         * @brief Constructor.
         * @param name Node name in the behavior tree.
         * @param config BehaviorTree.CPP node configuration.
         */
        LogMessageNode(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Defines the input port used by this node.
         * @return List of expected input ports.
         */
        static BT::PortsList providedPorts();

        /**
         * @brief Executes the node.
         * Logs the message and returns SUCCESS if input is valid, otherwise FAILURE.
         */
        BT::NodeStatus tick() override;
    };

} // namespace cr::bt::common

#endif // CR_BT_COMMON__LOG_MESSAGE_NODE_HPP_
