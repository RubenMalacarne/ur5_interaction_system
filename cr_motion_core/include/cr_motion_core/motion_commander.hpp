#ifndef CR_MOTION_CORE__MOTION_COMMANDER_HPP_
#define CR_MOTION_CORE__MOTION_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <future> // Necessario per std::shared_future e std::promise

namespace cr::motion_core
{
    // enumerato per gestire lo stato di un task asincrono
    enum MotionStatus
    {
        PENDING = 0,
        RUNNING = 1,
        SUCCEEDED = 2,
        FAILED = 3,
        CANCELLED = 4
    };

    class MotionCommander
    {
    public:
        // Costruttore: riceve il nodo ROS e i nomi dei gruppi MoveIt
        MotionCommander(
            const rclcpp::Node::SharedPtr &node,
            const std::string &arm_group_name = "arm_manipulator",
            const std::string &gripper_group_name = "gripper");

        // Distruttore: ferma eventuali esecuzioni in corso
        ~MotionCommander();

        rclcpp::Logger get_logger() const { return node_->get_logger(); }


        // --- METODI PER IL BRACCIO ---
        std::shared_future<MotionStatus> async_vertical_move(double z);
        std::shared_future<MotionStatus> async_horizontal_move(double x, double y);
        std::shared_future<MotionStatus> async_go_home();
        void cancel_arm_execution();
        MotionStatus get_arm_motion_status() const;

        // --- METODI PER IL GRIPPER ---

        /**
         * @brief Pianifica ed esegue in modo asincrono il movimento del gripper
         * al valore target specificato.
         * @param target_joint_value Il valore target del giunto del gripper.
         * @return Una shared_future per monitorare lo stato dell'operazione.
         *         Se un'operazione è già in corso o la pianificazione fallisce,
         *         restituisce una shared_future non valida.
         */
        std::shared_future<MotionStatus> async_set_gripper_joint(double target_joint_value);

        // Metodo per cancellare l'esecuzione del gripper
        void cancel_gripper_execution();

        // Metodo per ottenere lo stato del movimento del gripper
        MotionStatus get_gripper_motion_status() const;

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

        // Future per tracciare l'esecuzione asincrona
        mutable std::shared_future<MotionStatus> arm_task_future_;
        mutable std::shared_future<MotionStatus> gripper_task_future_;

        inline static const std::string CONTROLLED_GRIPPER_JOINT =
            "robotiq_85_left_knuckle_joint";

        // --- METODI HELPER PER IL BRACCIO ---
        std::shared_future<MotionStatus> async_cartesian_move(const geometry_msgs::msg::Pose& target_pose);
        bool plan_cartesian_path(const geometry_msgs::msg::Pose& target_pose, moveit_msgs::msg::RobotTrajectory& trajectory);
        bool waitForRobotState(double timeout_sec);
    };

} // namespace cr::motion_core

#endif // CR_MOTION_CORE__MOTION_COMMANDER_HPP_
