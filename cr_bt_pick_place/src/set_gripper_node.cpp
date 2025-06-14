#include "cr_bt_pick_place/set_gripper_node.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <cr_motion_core/motion_commander.hpp>

using namespace cr::bt::pick_place::nodes;
using BT::NodeStatus;

SetGripperNode::SetGripperNode(const std::string &name,
                               const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
    // Retrieve the ROS node and MotionCommander from the blackboard
    if (!config.blackboard->get("ros_node", nh_))
    {
        throw BT::RuntimeError("SetGripperNode: missing 'ros_node' on blackboard. Check your setupBT in the orchestrator.");
    }

    if (!config.blackboard->get("motion_commander", commander_))
    {
        throw BT::RuntimeError("SetGripperNode: missing 'motion_commander' on blackboard.");
    }
}

SetGripperNode::~SetGripperNode() = default;

BT::PortsList SetGripperNode::providedPorts()
{
    return {
        BT::InputPort<double>("target", "Gripper joint target [rad]")};
}

NodeStatus SetGripperNode::onStart()
{
    double target;

    if (!getInput("target", target))
    {
        RCLCPP_ERROR(nh_->get_logger(), "SetGripperNode '%s': missing required input [target]", name().c_str());
        return NodeStatus::FAILURE;
    }

    if (!commander_)
    {
        RCLCPP_ERROR(nh_->get_logger(), "SetGripperNode '%s': motion_commander not initialized.", name().c_str());
        return NodeStatus::FAILURE;
    }

    // Send non-blocking gripper command
    gripper_task_future_ = commander_->async_set_gripper_joint(target);

    if (!gripper_task_future_.valid())
    {
        RCLCPP_ERROR(nh_->get_logger(), "SetGripperNode '%s': Failed to start async gripper task.", name().c_str());
        return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(nh_->get_logger(), "SetGripperNode '%s': Sent gripper command (target %.3f rad).", name().c_str(), target);
    return NodeStatus::RUNNING;
}

NodeStatus SetGripperNode::onRunning()
{
    cr::motion_core::MotionStatus motion_status = commander_->get_gripper_motion_status();

    switch (motion_status)
    {
    case cr::motion_core::MotionStatus::RUNNING:
        return NodeStatus::RUNNING;

    case cr::motion_core::MotionStatus::SUCCEEDED:
        RCLCPP_INFO(nh_->get_logger(), "SetGripperNode '%s': Gripper task succeeded.", name().c_str());
        return NodeStatus::SUCCESS;

    case cr::motion_core::MotionStatus::CANCELLED:
        RCLCPP_WARN(nh_->get_logger(), "SetGripperNode '%s': Gripper task was cancelled.", name().c_str());
        return NodeStatus::FAILURE;

    case cr::motion_core::MotionStatus::FAILED:
    case cr::motion_core::MotionStatus::PENDING:
    default:
        RCLCPP_ERROR(nh_->get_logger(), "SetGripperNode '%s': Gripper task failed or returned unexpected status (%d).",
                     name().c_str(), static_cast<int>(motion_status));
        return NodeStatus::FAILURE;
    }
}

void SetGripperNode::onHalted()
{
    RCLCPP_WARN(nh_->get_logger(), "SetGripperNode '%s': Halted. Requesting gripper cancellation.", name().c_str());
    if (commander_)
    {
        commander_->cancel_gripper_execution();
    }
}
