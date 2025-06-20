/**
 * @file task_orchestrator.cpp
 * @brief Implementation of the TaskOrchestrator node for managing robotic workflows.
 */

#include "cr_task_orchestration/task_orchestrator.hpp"
#include <cr_interfaces/msg/object_info.hpp>

namespace cr {
namespace task_orchestration {
    /**
     * @brief Constructor for the TaskOrchestrator class.
     *
     * responsible for handling workflow execution.
     *
     * @param options Node options passed by ROS 2.
     */
    TaskOrchestrator::TaskOrchestrator(const rclcpp::NodeOptions &options)
        : Node("task_orchestrator", options)
    {
        using namespace std::placeholders;

        this->pick_client_ptr_ = rclcpp_action::create_client<Pick>(this, "cr/pick_action");
        this->place_client_ptr_ = rclcpp_action::create_client<Place>(this, "cr/place_action");

        this->get_object_info_client_ = this->create_client<cr_interfaces::srv::GetObjectInfo>("cr/get_object_info");

        this->freeze_scene_pub_ = this->create_publisher<cr_interfaces::msg::FreezeScene>("cr/freeze_scene", 10);

        this->execute_workflow_server_ptr_ = rclcpp_action::create_server<cr_interfaces::action::ExecuteWorkflow>(
            this,
            "cr/execute_workflow",
            std::bind(&TaskOrchestrator::handle_goal, this, _1, _2),
            std::bind(&TaskOrchestrator::handle_cancel, this, _1),
            std::bind(&TaskOrchestrator::handle_accepted, this, _1)
        );
    }


    //////////////////////////////////////////////////////
    //                  PARTE SERVER                    //
    //////////////////////////////////////////////////////
    /**
     * @brief Callback for handling incoming workflow execution goals.
     *
     * @param uuid Unique goal ID.
     * @param goal The goal message containing workflow parameters.
     * @return Goal response indicating whether the goal is accepted or rejected.
     */
    rclcpp_action::GoalResponse TaskOrchestrator::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteWorkflow::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received new workflow execution request: object_id=%d", goal->object_id);
        (void)uuid;

        if(is_busy_){
            RCLCPP_WARN(this->get_logger(), "Cannot accept new goal, busy executing another workflow.");
            return rclcpp_action::GoalResponse::REJECT;
        } else {
            
            // Avvisiamo che la scena deve essere freezata
            cr_interfaces::msg::FreezeScene msg;
            msg.freeze = true;
            freeze_scene_pub_->publish(msg);

            is_busy_ = true;
            RCLCPP_INFO(this->get_logger(), "Goal accepted.");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
    }
    
    /**
     * @brief Callback for handling goal cancellation.
     *
     * @param goal_handle Handle to the goal being cancelled.
     * @return Cancel response.
     */
    rclcpp_action::CancelResponse TaskOrchestrator::handle_cancel(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;

        // Avvisiamo che la scena non deve più essere freezata
        cr_interfaces::msg::FreezeScene msg;
        msg.freeze = false;
        freeze_scene_pub_->publish(msg);

        is_busy_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    
    /**
     * @brief Callback triggered when a goal is accepted.
     *
     * Starts a new thread to fetch object info before executing the workflow.
     *
     * @param goal_handle Handle to the accepted goal.
     */
    void  TaskOrchestrator::handle_accepted(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
    {
        using namespace std::placeholders;
        this->current_goal_handle_ = goal_handle;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&TaskOrchestrator::get_object_info, this, goal_handle)}.detach();

    }

    //////////////////////////////////////////////////////
    //                  OBJECT_INFO                     //
    //////////////////////////////////////////////////////
    /**
     * @brief Fetches object information from the service.
     *
     * @param goal_handle Handle to the current workflow goal.
     */
    void TaskOrchestrator::get_object_info(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle){
        auto request = std::make_shared<cr_interfaces::srv::GetObjectInfo::Request>();
        request->id = goal_handle->get_goal()->object_id;
    
        auto future = get_object_info_client_->async_send_request(request,
            [this, goal_handle](rclcpp::Client<cr_interfaces::srv::GetObjectInfo>::SharedFuture result_future)
            {
                if (result_future.get()->success)
                {
                    this->target_object_ = result_future.get()->object_info;
                    RCLCPP_INFO(this->get_logger(), "Received object info. Sending Pick goal.");
                    this->send_pick_goal();
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Object info not found. Aborting workflow.");

                    // Avvisiamo che la scena non deve più essere freezata
                    cr_interfaces::msg::FreezeScene msg;
                    msg.freeze = false;
                    freeze_scene_pub_->publish(msg);

                    is_busy_ = false;
                    goal_handle->abort(std::make_shared<ExecuteWorkflow::Result>());
                }
            }
        );
    }

    //////////////////////////////////////////////////////
    //                      PICK                        //
    //////////////////////////////////////////////////////
    /**
     * @brief Sends a pick goal to the pick action server.
     */
    void TaskOrchestrator::send_pick_goal()
    {
        using namespace std::placeholders;

        if(!this->pick_client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Pick Action Server not available after waiting");
            // TODO: deve in qualche modo fallire
        }

        auto pick_goal_msg = Pick::Goal();
        pick_goal_msg.object_info = target_object_;

        RCLCPP_INFO(this->get_logger(), "Sending pick goal...");

        auto send_pick_goal_options = rclcpp_action::Client<Pick>::SendGoalOptions();
        send_pick_goal_options.goal_response_callback = std::bind(&TaskOrchestrator::pick_goal_response_callback, this, _1);
        send_pick_goal_options.feedback_callback = std::bind(&TaskOrchestrator::pick_feedback_callback, this, _1, _2);
        send_pick_goal_options.result_callback = std::bind(&TaskOrchestrator::pick_result_callback, this, _1);
        this->pick_client_ptr_->async_send_goal(pick_goal_msg, send_pick_goal_options);
    }

    /**
     * @brief Callback for the pick goal response.
     */
    void TaskOrchestrator::pick_goal_response_callback(const GoalHandlePick::SharedPtr & goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Pick goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Pick goal accepted by server, waiting for result");
            // TODO: Gestire il fatto che non è più busy e sbloccare la scena 
        }
    }

    
    /**
     * @brief Callback for pick action feedback.
     */
    void TaskOrchestrator::pick_feedback_callback(
        GoalHandlePick::SharedPtr, 
        const std::shared_ptr<const Pick::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "[Pick Feedback] Percentage: %.2f | Message: %s", feedback->percentage, feedback->feedback_msg.c_str());
    }

    /**
     * @brief Callback for the result of the pick action.
     */
    void TaskOrchestrator::pick_result_callback(const GoalHandlePick::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_ERROR(this->get_logger(), "Pick goal succeeded!");
                send_place_goal();
                return;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Pick goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Pick goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
          }
    }


    //////////////////////////////////////////////////////
    //                     PLACE                        //
    //////////////////////////////////////////////////////
    /**
     * @brief Sends a place goal to the place action server.
     */
    void TaskOrchestrator::send_place_goal()
    {
        using namespace std::placeholders;

        if(!this->place_client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Place Action Server not available after waiting");
            // TODO: deve in qualche modo fallire
        }

        auto place_goal_msg = Place::Goal();
        place_goal_msg.object_info = target_object_;
        place_goal_msg.target_position.x = 0.300;
        place_goal_msg.target_position.y = 0.625;
        place_goal_msg.target_position.z = 0.866;

        RCLCPP_INFO(this->get_logger(), "Sending place goal...");

        auto send_place_goal_options = rclcpp_action::Client<Place>::SendGoalOptions();
        send_place_goal_options.goal_response_callback = std::bind(&TaskOrchestrator::place_goal_response_callback, this, _1);
        send_place_goal_options.feedback_callback = std::bind(&TaskOrchestrator::place_feedback_callback, this, _1, _2);
        send_place_goal_options.result_callback = std::bind(&TaskOrchestrator::place_result_callback, this, _1);
        this->place_client_ptr_->async_send_goal(place_goal_msg, send_place_goal_options);
    }

    
    /**
     * @brief Callback for the place goal response.
     */
    void TaskOrchestrator::place_goal_response_callback(const GoalHandlePlace::SharedPtr & goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Place goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Place goal accepted by server, waiting for result");
        }
    }

    /**
     * @brief Callback for place action feedback.
     */
    void TaskOrchestrator::place_feedback_callback(
        GoalHandlePlace::SharedPtr, 
        const std::shared_ptr<const Place::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "[Place Feedback] Percentage: %.2f | Message: %s", feedback->percentage, feedback->feedback_msg.c_str());
    }

    
    /**
     * @brief Callback for the result of the place action.
     */
    void TaskOrchestrator::place_result_callback(const GoalHandlePlace::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
                RCLCPP_ERROR(this->get_logger(), "Place goal succeeded!");
                
                cr_interfaces::msg::FreezeScene msg;
                msg.freeze = false;
                freeze_scene_pub_->publish(msg);
            
                is_busy_ = false;
            
                if (current_goal_handle_) {
                    auto result = std::make_shared<ExecuteWorkflow::Result>();
                    result->success = true;
                    result->msg = "Workflow completed successfully";
                    current_goal_handle_->succeed(result);
                    current_goal_handle_.reset();
                }
            
                return;
            }
            case rclcpp_action::ResultCode::ABORTED:
            {
                RCLCPP_ERROR(this->get_logger(), "Place goal was aborted");
                return;
            }
            case rclcpp_action::ResultCode::CANCELED:
            {
                RCLCPP_ERROR(this->get_logger(), "Place goal was canceled");
                return;
            }
            default:
            {
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
            }
        }
    }    

} // namespace task_orchestation
} // namespace cr