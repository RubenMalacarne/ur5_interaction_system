#include "cr_action_servers/pick_action_server.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/planning_interface/planning_interface.h>
/**
 * @file pick_action_server.cpp
 * @brief Implements a ROS 2 Action Server to perform a pick operation using MoveIt.
 *
 * This server listens for pick goals, plans pre-grasp and grasp trajectories,
 * manipulates the gripper, and manages collision and attachment services.
 */

using namespace std::placeholders;

namespace cr {
namespace action_servers {
    /**
     * @class PickActionServer
     * @brief Action server for executing pick operations using MoveIt and ROS 2 actions.
     */
    PickActionServer::PickActionServer(const rclcpp::NodeOptions &options)
        : Node("pick_action_server", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting PickActionServer...");

        // Config parameters
        this->declare_parameter("pre_approach_distance", 0.30);
        this->declare_parameter("approach_distance", 0.05);
        this->declare_parameter("retreat_offset", 0.20);

        this->get_parameter("pre_approach_distance", pre_approach_distance_);
        this->get_parameter("approach_distance", approach_distance_);
        this->get_parameter("retreat_offset", retreat_offset_);

        // Action Server
        action_server_ = rclcpp_action::create_server<Pick>(
            this,
            "cr/pick_action",
            std::bind(&PickActionServer::handle_goal, this, _1, _2),
            std::bind(&PickActionServer::handle_cancel, this, _1),
            std::bind(&PickActionServer::handle_accepted, this, _1)
        );

        // Allow Collision Client
        allow_collision_client_ = this->create_client<cr_interfaces::srv::AllowCollision>("/allow_collision");
        attach_object_client_ = this->create_client<cr_interfaces::srv::AttachObject>("/attach_object");

        RCLCPP_INFO(this->get_logger(), "PickActionServer is ready.");

    }
    /**
     * @brief Callback executed when a new goal is received.
     * @param uuid The unique goal identifier.
     * @param goal Shared pointer to the goal message.
     * @return GoalResponse indicating acceptance or rejection.
     */
    rclcpp_action::GoalResponse PickActionServer::handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Pick::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received goal request:\n - Center: [x: %.3f, y: %.3f, z: %.3f]\n - Size: [x: %.3f, y: %.3f, z: %.3f]",
            goal->object_info.center.x, goal->object_info.center.y, goal->object_info.center.z,
            goal->object_info.size.x, goal->object_info.size.y, goal->object_info.size.z
        );
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    
    /**
     * @brief Callback executed when a cancel request is received.
     * @param goal_handle Handle to the active goal.
     * @return CancelResponse indicating whether the goal can be canceled.
     */
    rclcpp_action::CancelResponse PickActionServer::handle_cancel(const std::shared_ptr<GoalHandlePick> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    
    /**
     * @brief Callback executed when a goal is accepted for execution.
     * @param goal_handle Handle to the accepted goal.
     */
    void PickActionServer::handle_accepted(const std::shared_ptr<GoalHandlePick> goal_handle)
    {
      std::thread{std::bind(&PickActionServer::execute, this, _1), goal_handle}.detach();
    }

    /**
     * @brief Main execution logic for the pick action. Performs each stage of the pick process:
     * pre-grasp height, pre-grasp approach, object approach, gripper close, and retreat.
     * Publishes feedback and sets result.
     * @param goal_handle Handle to the goal being executed.
     */
    // Execution of the main logic
    void PickActionServer::execute(const std::shared_ptr<GoalHandlePick> goal_handle)
    {
        if (!arm_group_)
        {
            arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "arm_manipulator");
        }
        if (!gripper_group_)
        {
            gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "gripper");
        }

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Pick::Feedback>();
        auto result = std::make_shared<Pick::Result>();
    
        const auto &object_info = goal->object_info;

        feedback->percentage = 10.0;
        feedback->feedback_msg = "Planning move above object...";
        goal_handle->publish_feedback(feedback);

        if(!reach_pre_grasp_height(object_info))
        {
            result->success = false;
            result->result_msg = "Failed to reach pre-grasp height.";
            goal_handle->abort(result);
            return;
        } 

        feedback->percentage = 15.0;
        feedback->feedback_msg = "Reaching Pre-Grasping pose";
        goal_handle->publish_feedback(feedback);

        if(!reach_pre_grasp(object_info))
        {
            result->success = false;
            result->result_msg = "Failed to reach pre-grasp pose.";
            goal_handle->abort(result);
            return;
        } 

        feedback->percentage = 30.0;
        feedback->feedback_msg = "Approaching object...";
        goal_handle->publish_feedback(feedback);

        if(!approach_object(object_info))
        {
            result->success = false;
            result->result_msg = "Failed to approach object.";
            goal_handle->abort(result);
            return;
        }

        feedback->percentage = 60.0;
        feedback->feedback_msg = "Closing gripper...";
        goal_handle->publish_feedback(feedback);

        if(!pick_object(object_info))
        {
            result->success = false;
            result->result_msg = "Failed to close gripper.";
            goal_handle->abort(result);
            return;
        }

        feedback->percentage = 80.0;
        feedback->feedback_msg = "Retreating...";
        goal_handle->publish_feedback(feedback);
    
        if(!retreat()){
            result->success = false;
            result->result_msg = "Failed to retreat.";
            goal_handle->abort(result);
            return;            
        }

        feedback->percentage = 100.0;
        feedback->feedback_msg = "Pick completed.";
        goal_handle->publish_feedback(feedback);

        result->success = true;
        result->result_msg = "Pick task complete successfully.";
        goal_handle->succeed(result);

    }

    /**
     * @brief Moves the robot end-effector to a height above the target object.
     * @param object_info The object to be picked.
     * @return True if the motion succeeds, false otherwise.
    */
    // Helper Methods
    bool PickActionServer::reach_pre_grasp_height(const cr_interfaces::msg::ObjectInfo &object_info)
    {
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z = object_info.center.z + object_info.size.z / 2.0 + pre_approach_distance_;
    
        RCLCPP_INFO(this->get_logger(),
            "object_info.center.z=%.3f  object_info.size.z=%.3f  approach_distance_=%.3f",
            object_info.center.z, object_info.size.z, approach_distance_);


        // 3. Debug: stampa le coordinate
        RCLCPP_INFO(
            this->get_logger(),
            "reach_pre_grasp_height:\n"
            "  Start pose: [x=%.3f, y=%.3f, z=%.3f]\n"
            "  Target pose: [x=%.3f, y=%.3f, z=%.3f]",
            start_pose.position.x, start_pose.position.y, start_pose.position.z,
            target_pose.position.x, target_pose.position.y, target_pose.position.z
        );
    
        // 4. Un solo waypoint (possiamo anche gestirne di più se serve)
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);
    
        // 5. Calcolo cartesian path
        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.01;
        double jump_threshold = 0.0;
    
        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            eef_step,
            jump_threshold,
            trajectory
        );
    
        RCLCPP_INFO(
            this->get_logger(),
            "  Cartesian path fraction: %.2f",
            fraction
        );
    
        // 6. Se il planner realizza meno di un 80% del percorso, consideriamo fallito
        if (fraction < 0.8)
        {
            RCLCPP_WARN(this->get_logger(),
                        "  Fraction is below threshold (%.2f). Aborting.", fraction);
            return false;
        }
    
        // 7. Se ok, prepariamo il plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
    
        // 8. Esegui il piano
        auto error_code = arm_group_->execute(plan);
        if (error_code != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(),
                        "  Execution of cartesian path failed (error code: %d)",
                        error_code.val);
            return false;
        }
    
        RCLCPP_INFO(this->get_logger(), "  Successfully reached pre-grasp height!");
        return true;
    }    

    
    /**
     * @brief Moves the end-effector above the object, in the XY plane.
     * @param object_info The object to be picked.
     * @return True if successful, false otherwise.
    */
    bool PickActionServer::reach_pre_grasp(const cr_interfaces::msg::ObjectInfo &object_info)
    {
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.x = object_info.center.x;
        target_pose.position.y = object_info.center.y;

        std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.01;
        double jump_threshold = 0.0;

        double fraction = arm_group_->computeCartesianPath(
            waypoints, 
            eef_step,
            jump_threshold,
            trajectory
        );

        if (fraction < 0.9)
            return false;
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        return arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }   

    /**
     * @brief Lowers the end-effector closer to the object to prepare for grasping.
     * @param object_info The object to be picked.
     * @return True if successful, false otherwise.
     */
    bool PickActionServer::approach_object(const cr_interfaces::msg::ObjectInfo &object_info)
    {
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z = object_info.center.z + object_info.size.z/2 + approach_distance_;

        std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.01;
        double jump_threshold = 0.0;

        double fraction = arm_group_->computeCartesianPath(
            waypoints, 
            eef_step,
            jump_threshold,
            trajectory
        );

        if (fraction < 0.9)
            return false;
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        return arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    /**
     * @brief Closes the gripper and attaches the object to the robot. Also calls collision/attachment services.
     * @param object_info The object to be picked.
     * @return True if the pick operation succeeds, false otherwise.
     */
    bool PickActionServer::pick_object(const cr_interfaces::msg::ObjectInfo &object_info){

        // // Consentiamo la collisione tra il gripper e l'oggetto
        auto request = std::make_shared<cr_interfaces::srv::AllowCollision::Request>();
        request->object_id = object_info.id;
        request->is_allowed = true;

        // Aspetta che il service sia disponibile
        while (!allow_collision_client_->wait_for_service(std::chrono::seconds(10)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
        }

        // Chiama il service in async
        auto future = allow_collision_client_->async_send_request(request);

        // // ⚠️ Usa un executor temporaneo locale per aspettare la risposta
        // rclcpp::executors::SingleThreadedExecutor local_exec;
        // local_exec.add_node(this->get_node_base_interface());

        // if (local_exec.spin_until_future_complete(future) == rclcpp::FutureReturnCode::SUCCESS)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Collision allowed successfully.");
        //     // puoi anche controllare `future.get()->success` se la tua risposta lo prevede
        // }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to call service allow_collision.");
        //     return false;
        // }

        std::this_thread::sleep_for(std::chrono::seconds(5));

        gripper_group_->setNamedTarget("gripper_close");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if(gripper_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            return false;
        gripper_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;

        // ATTACCHIAMO NUOVAMENTE L'OGGETTO ALLA SCENA DI MOVEIT
        // Consentiamo la collisione tra il gripper e l'oggetto
        auto attach_request = std::make_shared<cr_interfaces::srv::AttachObject::Request>();
        attach_request->object_id = object_info.id;
        attach_request->attach = true;

        // Aspetta che il service sia disponibile
        while (!attach_object_client_->wait_for_service(std::chrono::seconds(10))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
        }

        attach_object_client_->async_send_request(attach_request);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return true;
    }

    bool PickActionServer::retreat()
    {
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z += retreat_offset_;

        std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.01;
        double jump_threshold = 0.0;

        double fraction = arm_group_->computeCartesianPath(
            waypoints, 
            eef_step,
            jump_threshold,
            trajectory
        );

        if (fraction < 0.9)
            return false;
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        return arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }

} // namespace action_servers
} // namespace cr
