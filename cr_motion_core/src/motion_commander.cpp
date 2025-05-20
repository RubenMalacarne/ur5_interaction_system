#include "cr_motion_core/motion_commander.hpp"

namespace cr::motion_core
{

    MotionCommander::MotionCommander(
        const rclcpp::Node::SharedPtr &node,
        const std::string &arm_group_name,
        const std::string &gripper_group_name)
        : node_(node)
    {
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, gripper_group_name);
    }

    MotionCommander::~MotionCommander()
    {
        // Assicuriamoci di joinare il thread se è ancora vivo
        cancelSetGripper();
        if (gripper_thread_.joinable())
        {
            gripper_thread_.join();
        }
    }

    bool MotionCommander::sendSetGripper(double target)
    {
        if (gripper_running_.load())
        {
            RCLCPP_WARN(node_->get_logger(), "Gripper command already running");
            return false;
        }

        // 1) Pianifica (sincrono)
        gripper_group_->setJointValueTarget(CONTROLLED_GRIPPER_JOINT, target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool ok = static_cast<bool>(gripper_group_->plan(plan));
        if (!ok)
        {
            RCLCPP_WARN(node_->get_logger(), "Gripper planning failed");
            return false;
        }

        // 2) Lancia l’esecuzione in background
        gripper_running_ = true;
        gripper_success_ = false;
        gripper_thread_ = std::thread([this, plan = std::move(plan)]() mutable
                                      {
            auto result = gripper_group_->execute(plan);
            gripper_success_ = (result == moveit::core::MoveItErrorCode::SUCCESS);
            gripper_running_ = false; });

        return true;
    }

    int MotionCommander::setGripperStatus() const
    {
        if (gripper_running_)
        {
            return 0; // ancora in corso
        }
        return gripper_success_ ? 1 : 2;
    }

    void MotionCommander::cancelSetGripper()
    {
        if (gripper_running_)
        {
            // Ferma subito il controller
            gripper_group_->stop();
            gripper_running_ = false;
        }
    }

} // namespace cr::motion_core
