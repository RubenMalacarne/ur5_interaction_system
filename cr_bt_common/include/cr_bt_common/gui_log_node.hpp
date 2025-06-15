/**
 * @file gui_log_node.hpp
 * @brief Behavior Tree node that publishes structured logs to the GUI via a ROS topic.
 *
 * This SyncActionNode reads logging parameters from its input ports
 * and sends a structured message to the /cr/gui_log topic.
 */

#ifndef CR_BT_COMMON__GUI_LOG_NODE_HPP_
#define CR_BT_COMMON__GUI_LOG_NODE_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cr_interfaces/msg/log.hpp>

namespace cr::bt::common
{

    /**
     * @class GuiLog
     * @brief Publishes a GUI log message based on BT input ports.
     *
     * This node allows Behavior Trees to send GUI updates for progress tracking,
     * debugging and status reporting.
     */
    class GuiLog : public BT::SyncActionNode
    {
    public:
        /**
         * @brief Constructor. Initializes ROS publisher from the blackboard.
         * @param name Node name
         * @param config Behavior Tree node configuration
         */
        GuiLog(const std::string &name, const BT::NodeConfig &config);

        /**
         * @brief Declares the input ports used by this node.
         * @return List of required and optional BT ports.
         */
        static BT::PortsList providedPorts();

        /**
         * @brief Publishes the log message when the node is ticked.
         * @return Always returns SUCCESS
         */
        BT::NodeStatus tick() override;

    private:
        /// Shared publisher for GUI logs
        rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr pub_;
    };
} // namespace cr::bt::common

#endif // CR_BT_COMMON__GUI_LOG_NODE_HPP_