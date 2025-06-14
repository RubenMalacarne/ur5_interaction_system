#include "cr_bt_pick_place/arm_horizontal_move_node.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <cr_motion_core/motion_commander.hpp>

using namespace cr::bt::pick_place::nodes;
using BT::NodeStatus;

ArmHorizontalMoveNode::ArmHorizontalMoveNode(const std::string &name,
                                             const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
    // Retrieve ROS node and MotionCommander from blackboard
    if (!config.blackboard->get("ros_node", nh_))
    {
        throw BT::RuntimeError(name + ": missing 'ros_node' on blackboard.");
    }
    if (!config.blackboard->get("motion_commander", commander_))
    {
        throw BT::RuntimeError(name + ": missing 'motion_commander' on blackboard.");
    }

    RCLCPP_INFO(nh_->get_logger(), "%s initialized.", name.c_str());
}

ArmHorizontalMoveNode::~ArmHorizontalMoveNode() = default;

BT::PortsList ArmHorizontalMoveNode::providedPorts()
{
    return {
        BT::InputPort<double>("target_x", "Target X coordinate for horizontal movement [m]"),
        BT::InputPort<double>("target_y", "Target Y coordinate for horizontal movement [m]")
    };
}

NodeStatus ArmHorizontalMoveNode::onStart()
{
    double target_x, target_y;

    if (!getInput("target_x", target_x))
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: missing required input [target_x]", name().c_str());
        return NodeStatus::FAILURE;
    }
    if (!getInput("target_y", target_y))
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: missing required input [target_y]", name().c_str());
        return NodeStatus::FAILURE;
    }

    if (!commander_)
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: motion_commander not initialized.", name().c_str());
        return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(nh_->get_logger(), "%s: Requesting horizontal move to X=%.3f, Y=%.3f m.",
                name().c_str(), target_x, target_y);
    arm_task_future_ = commander_->async_horizontal_move(target_x, target_y);

    if (!arm_task_future_.valid())
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: Failed to start async_horizontal_move task.", name().c_str());
        return NodeStatus::FAILURE;
    }

    return NodeStatus::RUNNING;
}

NodeStatus ArmHorizontalMoveNode::onRunning()
{
    if (!commander_ || !arm_task_future_.valid())
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: Commander or future invalid in onRunning.", name().c_str());
        return NodeStatus::FAILURE;
    }

    // Query motion status
    cr::motion_core::MotionStatus motion_status = commander_->get_arm_motion_status();

    switch (motion_status)
    {
    case cr::motion_core::MotionStatus::RUNNING:
        return NodeStatus::RUNNING;
    case cr::motion_core::MotionStatus::SUCCEEDED:
        RCLCPP_INFO(nh_->get_logger(), "%s: Horizontal move task SUCCEEDED.", name().c_str());
        return NodeStatus::SUCCESS;
    case cr::motion_core::MotionStatus::CANCELLED:
        RCLCPP_WARN(nh_->get_logger(), "%s: Horizontal move task CANCELLED.", name().c_str());
        return NodeStatus::FAILURE;
    case cr::motion_core::MotionStatus::FAILED:
    case cr::motion_core::MotionStatus::PENDING:
    default:
        RCLCPP_ERROR(nh_->get_logger(), "%s: Horizontal move task FAILED or unexpected status (%d).",
                     name().c_str(), static_cast<int>(motion_status));
        return NodeStatus::FAILURE;
    }
}

void ArmHorizontalMoveNode::onHalted()
{
    RCLCPP_WARN(nh_->get_logger(), "%s: Halted. Requesting arm motion cancellation.", name().c_str());

    if (commander_)
    {
        commander_->cancel_arm_execution();
    }
}
