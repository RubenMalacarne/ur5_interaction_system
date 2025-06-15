/**
 * @file set_gripper_node.hpp
 * @brief BT node that commands the gripper to a desired target position.
 *
 * This BehaviorTree node wraps around a MotionCommander interface to
 * asynchronously open or close the gripper by sending a joint target position.
 *
 * It expects a "target" input (in radians) to define the desired gripper opening.
 */

#ifndef CR_BT_PICK_PLACE_NODES__SET_GRIPPER_NODE_HPP_
#define CR_BT_PICK_PLACE__SET_GRIPPER_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <future>

#include "cr_motion_core/motion_commander.hpp"

namespace cr::bt::pick_place::nodes
{

    /**
     * @class SetGripperNode
     * @brief Sends an asynchronous gripper command using MotionCommander.
     *
     * This BT stateful action node triggers a non-blocking command to move the gripper
     * to a specified joint position, and monitors its completion status.
     */
    class SetGripperNode : public BT::StatefulActionNode
    {
    public:
        /**
         * @brief Constructor.
         *
         * @param name Node name.
         * @param config Node configuration from the Behavior Tree.
         */
        SetGripperNode(const std::string &name,
                       const BT::NodeConfiguration &config);

        /// Default destructor
        ~SetGripperNode() override;

        /**
         * @brief Declares the input port required by this node.
         *
         * - target (double): Desired joint position in radians.
         */
        static BT::PortsList providedPorts();

    protected:
        /// Called once when the node starts running
        BT::NodeStatus onStart() override;

        /// Called repeatedly while the node is RUNNING
        BT::NodeStatus onRunning() override;

        /// Called when the node is halted externally
        void onHalted() override;

        /// Future to track async command
        std::shared_future<cr::motion_core::MotionStatus> gripper_task_future_;

        /// ROS node for logging
        rclcpp::Node::SharedPtr nh_;            
        
        /// Interface for motion commands
        std::shared_ptr<cr::motion_core::MotionCommander> commander_; 
    };

} // namespace cr::bt::pick_place::nodes

#endif // CR_BT_PICK_PLACE__SET_GRIPPER_NODE_HPP_
