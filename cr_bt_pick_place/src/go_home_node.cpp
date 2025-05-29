#include "cr_bt_pick_place/go_home_node.hpp"

using namespace cr::bt::pick_place::nodes;
using BT::NodeStatus;

GoHomeNode::GoHomeNode(const std::string &name, const BT::NodeConfiguration &config)
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

GoHomeNode::~GoHomeNode() = default;

NodeStatus GoHomeNode::onStart()
{
    if (!commander_)
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: motion_commander not initialized.", name().c_str());
        return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(nh_->get_logger(), "%s: Requesting to go home...",
                name().c_str());
    arm_task_future_ = commander_->async_go_home();

    if (!arm_task_future_.valid())
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: Failed to start async_go_home task.", name().c_str());
        return NodeStatus::FAILURE;
    }
    return NodeStatus::RUNNING;
}


NodeStatus GoHomeNode::onRunning()
{
    if (!commander_ || !arm_task_future_.valid())
    {
        RCLCPP_ERROR(nh_->get_logger(), "%s: Commander or future invalid in onRunning.", name().c_str());
        return NodeStatus::FAILURE;
    }

    cr::motion_core::MotionStatus motion_status = commander_->get_arm_motion_status();

    switch (motion_status)
    {
    case cr::motion_core::MotionStatus::RUNNING:
        return NodeStatus::RUNNING;
    case cr::motion_core::MotionStatus::SUCCEEDED:
        RCLCPP_INFO(nh_->get_logger(), "%s: Go Home task SUCCEEDED.", name().c_str());
        return NodeStatus::SUCCESS;
    case cr::motion_core::MotionStatus::CANCELLED:
        RCLCPP_WARN(nh_->get_logger(), "%s: Go Home task CANCELLED.", name().c_str());
        return NodeStatus::FAILURE;
    case cr::motion_core::MotionStatus::FAILED:
    case cr::motion_core::MotionStatus::PENDING:
    default:
        RCLCPP_ERROR(nh_->get_logger(), "%s: Go Home task FAILED or unexpected status (%d).",
                     name().c_str(), static_cast<int>(motion_status));
        return NodeStatus::FAILURE;
    }
}

void GoHomeNode::onHalted()
{
    RCLCPP_WARN(nh_->get_logger(), "%s: Halted. Requesting arm motion cancellation.", name().c_str());
    if (commander_)
    {
        commander_->cancel_arm_execution();
    }
}