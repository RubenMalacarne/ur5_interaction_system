/**
 * @file arm_horizontal_move_node.hpp
 * @brief BT node for commanding horizontal arm motion using MotionCommander.
 *
 * This node reads target X and Y coordinates from its input ports and issues an
 * asynchronous cartesian motion request through a shared MotionCommander instance.
 *
 * It is designed to be used within a Behavior Tree and exposes its internal state
 * via the standard BT lifecycle callbacks (onStart, onRunning, onHalted).
 */

#ifndef CR_BT_PICK_PLACE_NODES__ARM_HORIZONTAL_MOVE_NODE_HPP_
#define CR_BT_PICK_PLACE_NODES__ARM_HORIZONTAL_MOVE_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <future>
#include "cr_motion_core/motion_commander.hpp"

namespace cr::bt::pick_place::nodes
{

    /**
     * @class ArmHorizontalMoveNode
     * @brief BT node that triggers an asynchronous horizontal (X/Y) arm movement.
     *
     * The node expects a shared MotionCommander on the blackboard and publishes a non-blocking
     * request to move the robot's end-effector in the horizontal plane (X, Y).
     *
     * This action is managed using the BT::StatefulActionNode lifecycle.
     */
    class ArmHorizontalMoveNode : public BT::StatefulActionNode
    {
    public:
        /**
         * @brief Constructor. Retrieves required shared resources from the blackboard.
         */
        ArmHorizontalMoveNode(const std::string &name, const BT::NodeConfiguration &config);

        /**
         * @brief Destructor.
         */
        ~ArmHorizontalMoveNode() override;

        /**
         * @brief Defines input ports: target_x, target_y.
         */
        static BT::PortsList providedPorts();

    protected:
        /**
         * @brief Called once when the node starts execution.
         * Issues the horizontal move command asynchronously.
         */
        BT::NodeStatus onStart() override;

        /**
         * @brief Called repeatedly while the action is running.
         * Returns SUCCESS or FAILURE depending on the motion result.
         */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Called if the action is halted.
         * Sends a cancellation to the MotionCommander.
         */
        void onHalted() override;

        /// Future to track arm motion status.
        std::shared_future<cr::motion_core::MotionStatus> arm_task_future_; 

        /// ROS node (retrieved from blackboard).
        rclcpp::Node::SharedPtr nh_;      
        
        /// MotionCommander handle.
        std::shared_ptr<cr::motion_core::MotionCommander> commander_;       
    };

} // namespace cr::bt::pick_place::nodes

#endif // CR_BT_PICK_PLACE__ARM_HORIZONTAL_MOVE_NODE_HPP_
