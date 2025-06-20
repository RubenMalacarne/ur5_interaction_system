#include "cr_action_servers/place_action_server.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using namespace std::placeholders;
/**
 * @file place_action_server.cpp
 * @brief Implementation of the PlaceActionServer, a ROS 2 action server for placing objects using MoveIt.
*/

namespace cr {
namespace action_servers
{
    /**
     * @class PlaceActionServer
     * @brief ROS 2 Action Server that executes a place task using MoveIt motion planning and manipulation.
    */
    PlaceActionServer::PlaceActionServer(const rclcpp::NodeOptions &options)
    : Node("place_action_server", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting PlaceActionServer...");

        // Parameters config
        this->declare_parameter("approach_distance", 0.11);
        this->declare_parameter("pre_place_height", 1.50);

        this->get_parameter("approach_distance", approach_distance_);
        this->get_parameter("pre_place_height", pre_place_height_);

        // Action Server
        action_server_ = rclcpp_action::create_server<Place>(
            this,
            "cr/place_action",
            std::bind(&PlaceActionServer::handle_goal, this, _1, _2),
            std::bind(&PlaceActionServer::handle_cancel, this, _1),
            std::bind(&PlaceActionServer::handle_accepted, this, _1)
        );

        // Allow Collision Client
        allow_collision_client_ = this->create_client<cr_interfaces::srv::AllowCollision>("/allow_collision");

        // Attach Object Client
        attach_object_client_ = this->create_client<cr_interfaces::srv::AttachObject>("/attach_object");

        RCLCPP_INFO(this->get_logger(), "PlaceActionServer Started.");
    }

    /**
     * @brief Called when a new goal is received.
     * @param uuid Unique identifier for the goal.
     * @param goal Shared pointer to the goal message.
     * @return GoalResponse to accept or reject the goal.
     */
    rclcpp_action::GoalResponse PlaceActionServer::handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Place::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received goal request:\n - Target Position: [x: %.3f, y: %.3f, z: %.3f]",
            goal->target_position.x, goal->target_position.y, goal->target_position.z
        );
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    
    /**
     * @brief Called when a cancel request is received.
     * @param goal_handle Handle to the goal being canceled.
     * @return CancelResponse indicating whether the cancel request is accepted.
     */
    rclcpp_action::CancelResponse PlaceActionServer::handle_cancel(const std::shared_ptr<GoalHandlePlace> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    
    /**
     * @brief Called when a goal is accepted for execution.
     * @param goal_handle Handle to the accepted goal.
    */
    void PlaceActionServer::handle_accepted(const std::shared_ptr<GoalHandlePlace> goal_handle)
    {
        std::thread{std::bind(&PlaceActionServer::execute, this, _1), goal_handle}.detach();
    }

    // Execution of the main logic
    void PlaceActionServer::execute(const std::shared_ptr<GoalHandlePlace> goal_handle)
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
        auto feedback = std::make_shared<Place::Feedback>();
        auto result = std::make_shared<Place::Result>();
    
        const auto &object_info = goal->object_info;
        const auto &target_position = goal->target_position;

        feedback->percentage = 20.0;
        feedback->feedback_msg = "Lifting from table...";
        goal_handle->publish_feedback(feedback);

        if(!lift_from_table())
        {
            result->success = false;
            result->result_msg = "Failed to lift from table.";
            goal_handle->abort(result);
            return;
        } 

        feedback->percentage = 40.0;
        feedback->feedback_msg = "Moving to pre place pose...";
        goal_handle->publish_feedback(feedback);
    
        if(!move_to_pre_place_pose(target_position))
        {
            result->success = false;
            result->result_msg = "Failed to move to pre place pose.";
            goal_handle->abort(result);
            return;
        } 

        feedback->percentage = 60.0;
        feedback->feedback_msg = "Approaching place pose...";
        goal_handle->publish_feedback(feedback);
    
        if(!approach_place_pose(object_info, target_position))
        {
            result->success = false;
            result->result_msg = "Failed to approaching place pose.";
            goal_handle->abort(result);
            return;
        } 

        feedback->percentage = 80.0;
        feedback->feedback_msg = "Opening gripper...";
        goal_handle->publish_feedback(feedback);

        if(!open_gripper(object_info))
        {
            result->success = false;
            result->result_msg = "Failed to open gripper.";
            goal_handle->abort(result);
            return;
        } 

        feedback->percentage = 100.0;
        feedback->feedback_msg = "Place completed.";
        goal_handle->publish_feedback(feedback); 

        result->success = true;
        result->result_msg = "Place task complete successfully.";
        goal_handle->succeed(result);

    }

    /**
     * @brief Lifts the object vertically to a pre-defined height before placing.
     * @return True if successful, false otherwise.
     */
    bool PlaceActionServer::lift_from_table(){
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z = pre_place_height_;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.01;
        double jump_threshold = 0.0;

        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            eef_step,
            jump_threshold,
            trajectory
        );

        if (fraction < 0.8)
        {
            RCLCPP_WARN(this->get_logger(),
                        "  Fraction is below threshold (%.2f). Aborting.", fraction);
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
    
        auto error_code = arm_group_->execute(plan);
        if (error_code != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(),
                        "  Execution of cartesian path failed (error code: %d)",
                        error_code.val);
            return false;
        }
    
        RCLCPP_INFO(this->get_logger(), "  Successfully reached pre-place height!");
        return true;
    }

    
    /**
     * @brief Moves the robot above the target place position (XY plane).
     * @param point Target position to move above.
     * @return True if successful, false otherwise.
     */
    bool PlaceActionServer::move_to_pre_place_pose(const geometry_msgs::msg::Point point)
    {
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.x = point.x;
        target_pose.position.y = point.y;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.01;
        double jump_threshold = 0.0;

        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            eef_step,
            jump_threshold,
            trajectory
        );

        if (fraction < 0.8)
        {
            RCLCPP_WARN(this->get_logger(),
                        "  Fraction is below threshold (%.2f). Aborting.", fraction);
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
    
        auto error_code = arm_group_->execute(plan);
        if (error_code != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(),
                        "  Execution of cartesian path failed (error code: %d)",
                        error_code.val);
            return false;
        }
    
        RCLCPP_INFO(this->get_logger(), "  Successfully move to pre place pose!");
        return true;
    }

        
    /**
     * @brief Moves the robot downward toward the final place position.
     * @param object_info Information about the object being placed.
     * @param point Target placement coordinates.
     * @return True if successful, false otherwise.
     */
    bool PlaceActionServer::approach_place_pose(const cr_interfaces::msg::ObjectInfo object_info, const geometry_msgs::msg::Point point)
    {
        geometry_msgs::msg::Pose start_pose = arm_group_->getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z = point.z + object_info.size.z + approach_distance_;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.01;
        double jump_threshold = 0.0;

        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            eef_step,
            jump_threshold,
            trajectory
        );

        if (fraction < 0.8)
        {
            RCLCPP_WARN(this->get_logger(),
                        "  Fraction is below threshold (%.2f). Aborting.", fraction);
            return false;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
    
        auto error_code = arm_group_->execute(plan);
        if (error_code != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(),
                        "  Execution of cartesian path failed (error code: %d)",
                        error_code.val);
            return false;
        }
    
        RCLCPP_INFO(this->get_logger(), "Successfully approach place pose!");
        return true;
    }

        
    /**
     * @brief Opens the gripper and detaches the object in the MoveIt scene.
     * @param object_info Information about the object to detach.
     * @return True if successful, false otherwise.
     */
    bool PlaceActionServer::open_gripper(const cr_interfaces::msg::ObjectInfo object_info){
        gripper_group_->setNamedTarget("gripper_open");
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if(gripper_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
            return false;
        gripper_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;

        auto detach_request = std::make_shared<cr_interfaces::srv::AttachObject::Request>();
        detach_request->object_id = object_info.id;
        detach_request->attach = false;

        // Stacchiamo l'oggetto dal robot anche nella scena di MoveIt
        while (!attach_object_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_WARN(this->get_logger(), "Service not available, waiting again...");
        }

        attach_object_client_->async_send_request(detach_request);
        std::this_thread::sleep_for(std::chrono::seconds(2));

        return true;
    }



} // namespace action_servers
} // namespace cr