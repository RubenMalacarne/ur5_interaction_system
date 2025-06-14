/**
 * @file arm_vertical_move_node.hpp
 * @brief BT node for commanding vertical arm motion using MotionCommander.
 *
 * This node receives a target Z coordinate from its input port and issues an
 * asynchronous vertical movement command through a MotionCommander.
 *
 * It is designed to be used within a Behavior Tree and exposes its internal state
 * via the standard BT lifecycle callbacks (onStart, onRunning, onHalted).
 */

#ifndef CR_BT_PICK_PLACE_NODES__ARM_VERTICAL_MOVE_NODE_HPP_
#define CR_BT_PICK_PLACE_NODES__ARM_VERTICAL_MOVE_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <future>
#include <cr_motion_core/motion_commander.hpp>

namespace cr::bt::pick_place::nodes
{

    /**
     * @class ArmVerticalMoveNode
     * @brief BT node that triggers asynchronous vertical (Z-axis) movement of the robot arm.
     *
     * Requires a shared MotionCommander and ROS node instance on the blackboard.
     * The target Z position is provided as input and executed via async_vertical_move().
     */
    class ArmVerticalMoveNode : public BT::StatefulActionNode
    {
    public:
        /**
         * @brief Constructor. Retrieves MotionCommander and ROS node from the blackboard.
         */
        ArmVerticalMoveNode(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Destructor.
         */
        ~ArmVerticalMoveNode() override;

        /**
         * @brief Defines input ports: target_z.
         */
        static BT::PortsList providedPorts();

    protected:
        /**
         * @brief Called once on node start. Initiates the vertical movement.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called periodically while the node is RUNNING.
         * Monitors the status of the motion.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if the node is halted during execution.
         * Sends a cancellation command to the arm.
         */
        void onHalted() override;

        /// Future tracking the motion task.
        std::shared_future<cr::motion_core::MotionStatus> arm_task_future_;

        /// Shared ROS node from blackboard.
        rclcpp::Node::SharedPtr nh_;

        /// MotionCommander instance.
        std::shared_ptr<cr::motion_core::MotionCommander> commander_;
    };

} // namespace cr::bt::pick_place::nodes

#endif // CR_BT_PICK_PLACE_NODES__ARM_VERTICAL_MOVE_NODE_HPP_
