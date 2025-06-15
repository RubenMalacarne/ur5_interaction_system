/**
 * @file go_home_node.hpp
 * @brief BT node for sending the robot arm to the predefined "home" pose.
 *
 * This BT stateful action node uses MotionCommander to asynchronously perform
 * a vertical lift followed by a named MoveIt target ("home").
 * 
 * It is designed to be used within a Behavior Tree and exposes its internal state
 * via the standard BT lifecycle callbacks (onStart, onRunning, onHalted).
 */

#ifndef CR_BT_PICK_PLACE_NODES__GO_HOME_NODE_HPP_
#define CR_BT_PICK_PLACE_NODES__GO_HOME_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <future>

#include <cr_motion_core/motion_commander.hpp>

namespace cr::bt::pick_place::nodes
{

/**
 * @class GoHomeNode
 * @brief BT node that triggers the robot arm to return to its "home" pose.
 *
 * This node starts the async_go_home() command of the MotionCommander, which lifts
 * the arm slightly and then moves to the predefined home configuration.
 */
class GoHomeNode : public BT::StatefulActionNode
{
public:
    /**
     * @brief Constructor. Retrieves required shared resources from the blackboard.
     */
    GoHomeNode(const std::string &name, const BT::NodeConfiguration &config);

    /**
     * @brief Destructor.
     */
    ~GoHomeNode() override;

    /**
     * @brief Defines input/output ports (none required for this node).
     */
    static BT::PortsList providedPorts() { return {}; }

protected:
    /**
     * @brief Called once when the node starts. Triggers the go_home command.
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Called while the node is RUNNING. Monitors task status.
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Called when the node is halted. Cancels the arm movement.
     */
    void onHalted() override;

private:
    /// Future tracking arm motion
    std::shared_future<cr::motion_core::MotionStatus> arm_task_future_;

    /// ROS node handle
    rclcpp::Node::SharedPtr nh_;  
    
    /// MotionCommander instance
    std::shared_ptr<cr::motion_core::MotionCommander> commander_;
};

} // namespace cr::bt::pick_place::nodes

#endif // CR_BT_PICK_PLACE_NODES__GO_HOME_NODE_HPP_
