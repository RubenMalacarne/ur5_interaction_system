#include "cr_motion_core/motion_commander.hpp"

#include <thread> // std::thread
#include <future> // std::promise / std::future
#include <chrono> // std::chrono literals

using namespace std::chrono_literals;

namespace cr::motion_core
{

    //---------------------------------------------------------------------
    //  HELPER (private)
    //---------------------------------------------------------------------

    /**
     * @brief  Attende che il CurrentStateMonitor di MoveIt abbia ricevuto
     *         almeno un JointState "recente".
     *
     * @param  timeout_sec  secondi di timeout (default = 1.0)
     * @return true se lo stato è disponibile, false in caso di timeout.
     */
    bool MotionCommander::waitForRobotState(double timeout_sec)
    {
        // assicuriamoci che il monitor sia attivo
        arm_group_->startStateMonitor();
        gripper_group_->startStateMonitor();

        // MoveIt2 (Humble/IRON) → getCurrentState(timeout) restituisce nullptr se scade
        moveit::core::RobotStatePtr state_arm = arm_group_->getCurrentState(timeout_sec);
        moveit::core::RobotStatePtr state_group = gripper_group_->getCurrentState(timeout_sec);

        if (!state_arm || !state_group)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[MotionCommander] nessun /joint_states ricevuto entro %.2fs",
                         timeout_sec);
            return false;
        }
        return true;
    }

    //---------------------------------------------------------------------
    //  COSTRUTTORE / DISTRUTTORE
    //---------------------------------------------------------------------

    MotionCommander::MotionCommander(const rclcpp::Node::SharedPtr &node,
                                     const std::string &arm_group_name,
                                     const std::string &gripper_group_name)
        : node_(node)
    {
        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, arm_group_name);
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, gripper_group_name);

        // Avvia il monitor e aspetta che arrivi lo stato, al massimo 2 secondi
        waitForRobotState(5.0);

        RCLCPP_INFO(node_->get_logger(),
                    "MotionCommander ready for groups '%s' / '%s'",
                    arm_group_name.c_str(),
                    gripper_group_name.c_str());
    }

    MotionCommander::~MotionCommander()
    {
        cancel_gripper_execution();
    }

    //---------------------------------------------------------------------
    //  METODI BRACCIO
    //---------------------------------------------------------------------

    std::shared_future<MotionStatus> MotionCommander::async_vertical_move(double z)
    {
        // Assicuriamoci di avere una posa corrente valida
        if (!waitForRobotState(2.0))
        {
            std::promise<MotionStatus> p;
            p.set_value(MotionStatus::FAILED);
            return p.get_future().share();
        }

        geometry_msgs::msg::PoseStamped current_pose = arm_group_->getCurrentPose();
        geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.z = z;

        RCLCPP_INFO(node_->get_logger(),
                    "Comando verticale: x=%.3f y=%.3f z=%.3f",
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z);

        return async_cartesian_move(target_pose);
    }

    std::shared_future<MotionStatus> MotionCommander::async_horizontal_move(double x,
                                                                            double y)
    {
        if (!waitForRobotState(2.0))
        {
            std::promise<MotionStatus> p;
            p.set_value(MotionStatus::FAILED);
            return p.get_future().share();
        }

        geometry_msgs::msg::PoseStamped current_pose = arm_group_->getCurrentPose();
        geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.x = x;
        target_pose.position.y = y;

        RCLCPP_INFO(node_->get_logger(),
                    "Comando orizzontale: x=%.3f y=%.3f z=%.3f (z invariata)",
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z);

        return async_cartesian_move(target_pose);
    }

    void MotionCommander::cancel_arm_execution()
    {
        if (arm_task_future_.valid() &&
            arm_task_future_.wait_for(0s) == std::future_status::timeout)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "[MotionCommander] Stop esecuzione braccio in corso");
            arm_group_->stop();
        }
    }

    MotionStatus MotionCommander::get_arm_motion_status() const
    {
        if (!arm_task_future_.valid())
            return MotionStatus::PENDING;

        if (arm_task_future_.wait_for(0s) == std::future_status::timeout)
            return MotionStatus::RUNNING;

        MotionStatus status = arm_task_future_.get();
        arm_task_future_ = std::shared_future<MotionStatus>(); // invalida
        return status;
    }

    std::shared_future<MotionStatus> MotionCommander::async_cartesian_move(
        const geometry_msgs::msg::Pose &target_pose)
    {
        // se c'è già un task in corso => ignora
        if (arm_task_future_.valid() &&
            arm_task_future_.wait_for(0s) == std::future_status::timeout)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Richiesta di movimento ignorata: braccio occupato");
            return std::shared_future<MotionStatus>();
        }

        // Pianifica
        moveit_msgs::msg::RobotTrajectory traj;
        if (!plan_cartesian_path(target_pose, traj))
        {
            std::promise<MotionStatus> p;
            p.set_value(MotionStatus::FAILED);
            arm_task_future_ = p.get_future().share();
            return arm_task_future_;
        }

        // Promise/Future per il task asincrono
        std::promise<MotionStatus> task_promise;
        arm_task_future_ = task_promise.get_future().share();

        std::thread([this, traj = std::move(traj), p = std::move(task_promise)]() mutable
                    {
        auto ec = arm_group_->execute(traj);
        p.set_value(ec == moveit::core::MoveItErrorCode::SUCCESS ? MotionStatus::SUCCEEDED
                                                                : MotionStatus::FAILED); })
            .detach();

        return arm_task_future_;
    }

    bool MotionCommander::plan_cartesian_path(const geometry_msgs::msg::Pose &target_pose,
                                              moveit_msgs::msg::RobotTrajectory &trajectory)
    {
        std::vector<geometry_msgs::msg::Pose> wps{target_pose};
        const double eef_step = 0.01;
        const double jump_thr = 0.0;

        double fraction = arm_group_->computeCartesianPath(wps, eef_step, jump_thr, trajectory);

        RCLCPP_INFO(node_->get_logger(),
                    "Cartesian path coverage: %.1f%%",
                    fraction * 100.0);

        return fraction > 0.9;
    }

    //---------------------------------------------------------------------
    //  METODI GRIPPER (invariati)
    //---------------------------------------------------------------------

    std::shared_future<MotionStatus> MotionCommander::async_set_gripper_joint(double target)
    {
        if (gripper_task_future_.valid() &&
            gripper_task_future_.wait_for(0s) == std::future_status::timeout)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Gripper già in esecuzione: richiesta ignorata");
            return std::shared_future<MotionStatus>();
        }

        gripper_group_->setJointValueTarget(CONTROLLED_GRIPPER_JOINT, target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (gripper_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        {
            std::promise<MotionStatus> p;
            p.set_value(MotionStatus::FAILED);
            gripper_task_future_ = p.get_future().share();
            return gripper_task_future_;
        }

        std::promise<MotionStatus> task_promise;
        gripper_task_future_ = task_promise.get_future().share();

        std::thread([this, plan = std::move(plan), p = std::move(task_promise)]() mutable
                    {
        auto ec = gripper_group_->execute(plan);
        p.set_value(ec == moveit::core::MoveItErrorCode::SUCCESS ? MotionStatus::SUCCEEDED
                                                                : MotionStatus::FAILED); })
            .detach();

        return gripper_task_future_;
    }

    void MotionCommander::cancel_gripper_execution()
    {
        if (gripper_task_future_.valid() &&
            gripper_task_future_.wait_for(0s) == std::future_status::timeout)
        {
            gripper_group_->stop();
        }
    }

    MotionStatus MotionCommander::get_gripper_motion_status() const
    {
        if (!gripper_task_future_.valid())
            return MotionStatus::PENDING;

        if (gripper_task_future_.wait_for(0s) == std::future_status::timeout)
            return MotionStatus::RUNNING;

        MotionStatus status = gripper_task_future_.get();
        gripper_task_future_ = std::shared_future<MotionStatus>();
        return status;
    }

} // namespace cr::motion_core
