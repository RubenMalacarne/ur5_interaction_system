/**
 * @file check_blackboard_key_node.hpp
 * @brief Defines a Behavior Tree condition node that checks if a given key exists in the blackboard.
 *
 * This node is designed to be reusable across Behavior Trees and packages.
 * It returns SUCCESS if the specified key exists in the blackboard, FAILURE otherwise.
 *
 * Input ports:
 * - "key" (std::string): The name of the key to look for in the blackboard.
 */

#ifndef CR_BT_COMMON__CHECK_BLACKBOARD_KEY_NODE_HPP_
#define CR_BT_COMMON__CHECK_BLACKBOARD_KEY_NODE_HPP_

#include <string>
#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>

namespace cr::bt::common
{
    /**
     * @class CheckBlackboardKeyNode
     * @brief A condition node that checks for the existence of a key in the Behavior Tree blackboard.
     */
    class CheckBlackboardKeyNode : public BT::ConditionNode
    {
    public:
        /**
         * @brief Constructor.
         * @param name Node name in the Behavior Tree.
         * @param config Configuration object containing ports and blackboard reference.
         */
        CheckBlackboardKeyNode(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Returns the list of input ports for this node.
         * @return List of ports including the required input key.
         */
        static BT::PortsList providedPorts();

        /**
         * @brief Main tick method called at runtime by the Behavior Tree engine.
         * @return SUCCESS if the key is present in the blackboard, FAILURE otherwise.
         */
        BT::NodeStatus tick() override;

    private:
        /**
         * @brief Retrieves a logger from the blackboard if available, otherwise uses a default logger.
         * @return A valid rclcpp::Logger instance.
         */
        rclcpp::Logger getLogger();
    };

} // namespace cr::bt::common

#endif // CR_BT_COMMON__CHECK_BLACKBOARD_KEY_NODE_HPP_
