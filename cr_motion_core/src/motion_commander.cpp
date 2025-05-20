#include "cr_motion_core/motion_commander.hpp"
#include <thread> // Necessario per std::thread
#include <future> // Necessario per std::promise

namespace cr::motion_core
{

    MotionCommander::MotionCommander(
        const rclcpp::Node::SharedPtr &node,
        const std::string &arm_group_name,
        const std::string &gripper_group_name)
        : node_(node)
    {
        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, arm_group_name);
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, gripper_group_name);

        RCLCPP_INFO(node_->get_logger(), "MotionCommander initialized for '%s' and '%s' groups.", arm_group_name.c_str(), gripper_group_name.c_str());
    }

    MotionCommander::~MotionCommander()
    {
        cancel_gripper_execution();
    }

    // -- METODI PER IL BRACCIO ---
    std::shared_future<MotionStatus> MotionCommander::async_vertical_move(double z)
    {
        // Modifichiamo solo Z della posa corrente
        geometry_msgs::msg::PoseStamped current_pose = arm_group_->getCurrentPose();
        geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.z = z;
        return async_cartesian_move(target_pose);
    }

    std::shared_future<MotionStatus> MotionCommander::async_horizontal_move(double x, double y)
    {
        // Modifichiamo solo X e Y della posa corrente
        geometry_msgs::msg::PoseStamped current_pose = arm_group_->getCurrentPose();
        geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        return async_cartesian_move(target_pose);
    }

    void MotionCommander::cancel_arm_execution()
    {
        // Verifichiamo se c'è un'esecuzione in corso
        if (arm_task_future_.valid() && arm_task_future_.wait_for(std::chrono::seconds(0)) == std::future_status::timeout)
        {
            RCLCPP_INFO(node_->get_logger(), "Richiesta di interruzione dell'esecuzione del braccio.");
            arm_group_->stop();
        }
        else if (arm_task_future_.valid() && arm_task_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            RCLCPP_INFO(node_->get_logger(), "Richiesta di interruzione del braccio, ma il task era già completato.");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Richiesta di interruzione del braccio, ma nessun task era in corso.");
        }
    }

    MotionStatus MotionCommander::get_arm_motion_status() const
    {
        if (!arm_task_future_.valid())
        {
            return MotionStatus::PENDING;
        }

        auto status = arm_task_future_.wait_for(std::chrono::seconds(0));

        if (status == std::future_status::timeout)
        {
            return MotionStatus::RUNNING;
        }
        else
        {
            MotionStatus result_status = arm_task_future_.get();
            arm_task_future_ = std::shared_future<MotionStatus>();
            return result_status;
        }
    }

    std::shared_future<MotionStatus> MotionCommander::async_cartesian_move(const geometry_msgs::msg::Pose &target_pose)
    {
        // Verifica se è già in corso un'esecuzione del braccio
        if (arm_task_future_.valid() && arm_task_future_.wait_for(std::chrono::seconds(0)) == std::future_status::timeout)
        {
            RCLCPP_WARN(node_->get_logger(), "Esecuzione del braccio già in corso. Richiesta ignorata.");
            return std::shared_future<MotionStatus>();
        }

        // Pianifica il percorso cartesiano
        moveit_msgs::msg::RobotTrajectory trajectory;
        bool success = plan_cartesian_path(target_pose, trajectory);

        if (!success)
        {
            RCLCPP_WARN(node_->get_logger(), "Pianificazione del movimento cartesiano fallita.");
            std::promise<MotionStatus> promise;
            promise.set_value(MotionStatus::FAILED);
            arm_task_future_ = promise.get_future().share();
            return arm_task_future_;
        }

        // Crea una promise e la future associata
        std::promise<MotionStatus> task_promise;
        arm_task_future_ = task_promise.get_future().share();

        // Avvia l'esecuzione in un thread separato
        std::thread([this, trajectory = std::move(trajectory), promise = std::move(task_promise)]() mutable
                    {
        // Esegue la traiettoria
        moveit::core::MoveItErrorCode result_code = arm_group_->execute(trajectory);

        // Converte il risultato MoveIt in MotionStatus
        MotionStatus final_status;
        if (result_code == moveit::core::MoveItErrorCode::SUCCESS) {
            final_status = MotionStatus::SUCCEEDED;
        } else {
            final_status = MotionStatus::FAILED;
        }

        // Imposta il risultato nel promise
        promise.set_value(final_status);

        RCLCPP_INFO(node_->get_logger(), "Esecuzione del movimento cartesiano completata con codice: %d, stato finale: %d.",
                   result_code.val, static_cast<int>(final_status)); })
            .detach();

        RCLCPP_INFO(node_->get_logger(), "Avviata esecuzione asincrona del movimento cartesiano.");
        return arm_task_future_;
    }

    bool MotionCommander::plan_cartesian_path(const geometry_msgs::msg::Pose &target_pose, moveit_msgs::msg::RobotTrajectory &trajectory)
    {
        geometry_msgs::msg::PoseStamped current_pose = arm_group_->getCurrentPose();
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        double eef_step = 0.01;
        double jump_threshold = 0.0; 

        double fraction = arm_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        RCLCPP_INFO(node_->get_logger(), "Pianificazione percorso cartesiano: %.2f%% di copertura del percorso.",
                    fraction * 100.0);

        return fraction > 0.9;
    }

    // --- METODI PER IL GRIPPER ---

    std::shared_future<MotionStatus> MotionCommander::async_set_gripper_joint(double target_joint_value)
    {
        // Verifica se è già in corso un'esecuzione del gripper
        if (gripper_task_future_.valid() && gripper_task_future_.wait_for(std::chrono::seconds(0)) == std::future_status::timeout)
        {
            RCLCPP_WARN(node_->get_logger(), "Esecuzione del gripper già in corso. Richiesta ignorata.");
            return std::shared_future<MotionStatus>(); // Ritorna una shared_future non valida
        }

        // 1. Pianifica il movimento del gripper (sincrono)
        gripper_group_->setJointValueTarget(CONTROLLED_GRIPPER_JOINT, target_joint_value);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(gripper_group_->plan(plan));

        if (!success)
        {
            RCLCPP_WARN(node_->get_logger(), "Pianificazione del movimento del gripper fallita.");
            // Pianificazione fallita, impostiamo subito lo stato a FAILED
            std::promise<MotionStatus> promise;
            promise.set_value(MotionStatus::FAILED);
            gripper_task_future_ = promise.get_future().share();
            return gripper_task_future_;
        }

        // 2. Crea una promise e la future associata per l'intero task (pianificazione + esecuzione)
        std::promise<MotionStatus> task_promise;
        gripper_task_future_ = task_promise.get_future().share();

        // 3. Avvia l'esecuzione del piano in un thread separato
        // Usiamo un thread separato perché MoveIt execute è bloccante
        std::thread([this, plan = std::move(plan), promise = std::move(task_promise)]() mutable
                    {
            // Esegue il piano
            moveit::core::MoveItErrorCode result_code = gripper_group_->execute(plan);

            // Converte il risultato MoveIt in MotionStatus
            MotionStatus final_status;
            if (result_code == moveit::core::MoveItErrorCode::SUCCESS) {
                final_status = MotionStatus::SUCCEEDED;
            } else {
                final_status = MotionStatus::FAILED;
            }

            // Imposta il risultato nel promise
            promise.set_value(final_status);

            RCLCPP_INFO(node_->get_logger(), "Esecuzione del gripper completata con codice: %d, stato finale: %d.", result_code.val, static_cast<int>(final_status)); })
            .detach(); // Stacca il thread

        RCLCPP_INFO(node_->get_logger(), "Avviata esecuzione asincrona del gripper.");
        return gripper_task_future_;
    }

    void MotionCommander::cancel_gripper_execution()
    {
        // Verifica se c'è un'esecuzione in corso monitorata dalla future
        if (gripper_task_future_.valid() && gripper_task_future_.wait_for(std::chrono::seconds(0)) == std::future_status::timeout)
        {
            RCLCPP_INFO(node_->get_logger(), "Richiesta di interruzione dell'esecuzione del gripper.");
            // MoveIt stop() è non bloccante
            gripper_group_->stop();
        }
        else if (gripper_task_future_.valid() && gripper_task_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            RCLCPP_INFO(node_->get_logger(), "Richiesta di interruzione del gripper, ma il task era già completato.");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Richiesta di interruzione del gripper, ma nessun task era in corso.");
        }
    }

    MotionStatus MotionCommander::get_gripper_motion_status() const
    {
        // Verifica se c'è una future valida
        if (!gripper_task_future_.valid())
        {
            // Nessuna esecuzione in corso o completata
            return MotionStatus::PENDING;
        }

        // Interroga lo stato della future senza bloccare
        auto status = gripper_task_future_.wait_for(std::chrono::seconds(0));

        if (status == std::future_status::timeout)
        {
            // Esecuzione ancora in corso
            return MotionStatus::RUNNING;
        }
        else
        {
            MotionStatus result_status = gripper_task_future_.get();
            // Invalida la future una volta che il risultato è stato ottenuto
            gripper_task_future_ = std::shared_future<MotionStatus>();
            return result_status;
        }
    }

} // namespace cr::motion_core
