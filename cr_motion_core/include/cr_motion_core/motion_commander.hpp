/**
 * @file motion_commander.hpp
 * @brief Abstraction layer over MoveIt MoveGroupInterface for arm and gripper.
 *
 * This module wraps MoveGroupInterface behind a simple, asynchronous API
 * that returns std::shared_future objects. It is designed to fit neatly into
 * reactive frameworks (such as behaviour trees), where motions must be launched,
 * monitored, and possibly cancelled without blocking the main control flow.
 */

#ifndef CR_MOTION_CORE__MOTION_COMMANDER_HPP_
#define CR_MOTION_CORE__MOTION_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <future>

namespace cr::motion_core
{

    /**
     * @brief Enum representing the status of an asynchronous motion command.
     */
    enum MotionStatus
    {
        PENDING = 0,
        RUNNING = 1,
        SUCCEEDED = 2,
        FAILED = 3,
        CANCELLED = 4
    };

    /**
     * @class MotionCommander
     * @brief High-level asynchronous motion interface for robot arm and gripper using MoveIt.
     *
     * This class abstracts MoveIt’s MoveGroupInterface to offer non-blocking motion commands
     * via std::shared_future. It simplifies integration into reactive or concurrent control systems,
     * where motion tasks need to run asynchronously while being monitored or cancelled.
     */
    class MotionCommander
    {
    public:
        /**
         * @brief Constructor.
         * Initializes both arm and gripper MoveGroupInterfaces and starts state monitoring.
         *
         * @param node Shared ROS 2 node.
         * @param arm_group_name MoveIt group name for the arm.
         * @param gripper_group_name MoveIt group name for the gripper.
         */
        MotionCommander(
            const rclcpp::Node::SharedPtr &node,
            const std::string &arm_group_name = "arm_manipulator",
            const std::string &gripper_group_name = "gripper");

        /**
         * @brief Destructor.
         * Cancels any ongoing gripper motion.
         */
        ~MotionCommander();

        rclcpp::Logger get_logger() const { return node_->get_logger(); }

        // ------------------- Arm Methods -------------------

        /**
         * @brief Asynchronously moves the arm vertically to the given Z.
         */
        std::shared_future<MotionStatus> async_vertical_move(double z);

        /**
         * @brief Asynchronously moves the arm in X/Y, keeping Z unchanged.
         */
        std::shared_future<MotionStatus> async_horizontal_move(double x, double y);

        /**
         * @brief Moves the arm to the "home" position after a vertical lift.
         */
        std::shared_future<MotionStatus> async_go_home();

        /**
         * @brief Cancels current arm motion if one is running.
         */
        void cancel_arm_execution();

        /**
         * @brief Returns the current arm motion status.
         */
        MotionStatus get_arm_motion_status() const;

        // ------------------ Gripper Methods ------------------

        /**
         * @brief Asynchronously sends a gripper joint to the target position.
         *
         * If a previous task is running, the new request is ignored.
         *
         * @param target_joint_value Desired value for the gripper joint.
         * @return A future representing the result of the action.
         */
        std::shared_future<MotionStatus> async_set_gripper_joint(double target_joint_value);

        /**
         * @brief Cancels current gripper motion, if active.
         */
        void cancel_gripper_execution();

        /**
         * @brief Returns the current gripper motion status.
         */
        MotionStatus get_gripper_motion_status() const;

    private:
        /**
         * @brief Plans and executes a Cartesian move asynchronously.
         */
        std::shared_future<MotionStatus> async_cartesian_move(const geometry_msgs::msg::Pose &target_pose);

        /**
         * @brief Plans a Cartesian path to the target pose.
         */
        bool plan_cartesian_path(const geometry_msgs::msg::Pose &target_pose,
                                 moveit_msgs::msg::RobotTrajectory &trajectory);

        /**
         * @brief Waits until robot state is available.
         * @param timeout_sec Max seconds to wait for valid joint states.
         */
        bool waitForRobotState(double timeout_sec);
        /// Shared ROS 2 node used for logging and clock access
        rclcpp::Node::SharedPtr node_;

        /// MoveGroup interface for the arm group
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;

        /// MoveGroup interface for the gripper group
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

        /// Shared future representing current arm task execution
        mutable std::shared_future<MotionStatus> arm_task_future_;

        /// Shared future representing current gripper task execution
        mutable std::shared_future<MotionStatus> gripper_task_future_;

        /// Name of the single actuated joint in the Robotiq-85 gripper; all other gripper joints mimic this one
        inline static const std::string CONTROLLED_GRIPPER_JOINT = "robotiq_85_left_knuckle_joint";
    };

} // namespace cr::motion_core

#endif // CR_MOTION_CORE__MOTION_COMMANDER_HPP_
