#ifndef CR_MOTION_CORE__MOTION_COMMANDER_HPP_
#define CR_MOTION_CORE__MOTION_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <atomic>

namespace cr::motion_core
{
    // enumerato per gestire lo stato di un task asincrono
    enum MotionStatus {
        PENDING = 0,
        RUNNING = 1,
        SUCCEEDED = 2,
        FAILED = 3,
        CANCELLED = 4
    };

    class MotionCommander
    {
    public:
        MotionCommander(
            const rclcpp::Node::SharedPtr &node,
            const std::string &arm_group_name = "arm_manipulator",
            const std::string &gripper_group_name = "gripper");

        ~MotionCommander();

        rclcpp::Logger get_logger() const { return node_->get_logger(); }

        /// Invia l’ordine di chiudere/aprire il gripper in background
        bool sendSetGripper(double target_joint_value);

        /// 0 = in corso, 1 = successo, 2 = fallimento
        int setGripperStatus() const;

        /// Ferma subito il gripper (chiamato da onHalted())
        void cancelSetGripper();

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

        // Thread interno per il gripper
        std::thread gripper_thread_;
        std::atomic<bool> gripper_running_{false};
        std::atomic<bool> gripper_success_{false};

        inline static const std::string CONTROLLED_GRIPPER_JOINT =
            "robotiq_85_left_knuckle_joint";
    };

} // namespace cr::motion_core

#endif // CR_MOTION_CORE__MOTION_COMMANDER_HPP_
