#include "cr_bt_pick_place/arm_vertical_move_node.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <cr_motion_core/motion_commander.hpp>

using namespace cr::bt::pick_place;
using BT::NodeStatus;

ArmVerticalMoveNode::ArmVerticalMoveNode(const std::string &name,
                                         const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
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

ArmVerticalMoveNode::~ArmVerticalMoveNode() = default;

BT::PortsList ArmVerticalMoveNode::providedPorts()
{
    return {
        BT::InputPort<double>("target_z", "Target Z coordinate for vertical movement [m]")};
}

NodeStatus ArmVerticalMoveNode::onStart()
{
    double target_z;

    if (!getInput("target_z", target_z))
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: missing required input [target_z]", name().c_str());
        return NodeStatus::FAILURE;
    }

    if (!commander_)
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: motion_commander not initialized.", name().c_str());
        return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(nh_->get_logger(), "%s: Requesting vertical move to Z = %.3f m.", name().c_str(), target_z);
    arm_task_future_ = commander_->async_vertical_move(target_z);

    if (!arm_task_future_.valid())
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: Failed to start async_vertical_move task.", name().c_str());
        return NodeStatus::FAILURE;
    }
    return NodeStatus::RUNNING;
}

NodeStatus ArmVerticalMoveNode::onRunning()
{
    if (!commander_ || !arm_task_future_.valid())
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: Commander or future invalid in onRunning.", name().c_str());
        return NodeStatus::FAILURE;
    }

    // Usa get_arm_motion_status() perché questo nodo controlla il braccio
    cr::motion_core::MotionStatus motion_status = commander_->get_arm_motion_status();

    switch (motion_status)
    {
    case cr::motion_core::MotionStatus::RUNNING:
        return NodeStatus::RUNNING;
    case cr::motion_core::MotionStatus::SUCCEEDED:
        RCLCPP_INFO(nh_->get_logger(), "%s: Vertical move task SUCCEEDED.", name().c_str());
        return NodeStatus::SUCCESS;
    case cr::motion_core::MotionStatus::CANCELLED:
        RCLCPP_WARN(nh_->get_logger(), "%s: Vertical move task CANCELLED.", name().c_str());
        return NodeStatus::FAILURE; // O SUCCESS se la cancellazione è un esito accettabile in certi casi
    case cr::motion_core::MotionStatus::FAILED:
    case cr::motion_core::MotionStatus::PENDING: // PENDING non dovrebbe accadere qui se onStart ha avuto successo
    default:
        RCLCPP_ERROR(nh_->get_logger(), "%s: Vertical move task FAILED or unexpected status (%d).",
                     name().c_str(), static_cast<int>(motion_status));
        return NodeStatus::FAILURE;
    }
}

void ArmVerticalMoveNode::onHalted()
{
    RCLCPP_WARN(nh_->get_logger(), "%s: Halted. Requesting arm motion cancellation.", name().c_str());
    if (commander_)
    {
        // Usa cancel_arm_execution()
        commander_->cancel_arm_execution();
    }
}
