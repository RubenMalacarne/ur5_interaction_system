#include "cr_task_orchestration/task_orchestrator.hpp"
#include <cr_interfaces/msg/object_info.hpp>

namespace cr {
namespace task_orchestration {

    TaskOrchestrator::TaskOrchestrator(const rclcpp::NodeOptions &options)
        : Node("task_orchestrator", options)
    {
        using namespace std::placeholders;

        this->pick_client_ptr_ = rclcpp_action::create_client<Pick>(this, "cr/pick_action");
        this->place_client_ptr_ = rclcpp_action::create_client<Place>(this, "cr/place_action");

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

    rclcpp_action::GoalResponse TaskOrchestrator::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteWorkflow::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received new workflow execution request: object_id=%d", goal->object_id);
        (void)uuid;

        if(is_busy_){
            RCLCPP_WARN(this->get_logger(), "Cannot accept new goal, busy executing another workflow.");
            return rclcpp_action::GoalResponse::REJECT;
        } else {
            is_busy_ = true;
            RCLCPP_INFO(this->get_logger(), "Goal accepted.");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
    }

    rclcpp_action::CancelResponse TaskOrchestrator::handle_cancel(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        is_busy_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void  TaskOrchestrator::handle_accepted(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&TaskOrchestrator::send_pick_goal, this, goal_handle)}.detach();

    }

    //////////////////////////////////////////////////////
    //                      PICK                        //
    //////////////////////////////////////////////////////
    void TaskOrchestrator::send_pick_goal(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
    {
        using namespace std::placeholders;

        current_object_id_ = goal_handle->get_goal()->object_id;

        if(!this->pick_client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Pick Action Server not available after waiting");
            // TODO: deve in qualche modo fallire
        }

        // TODO: tutto questo poi dovrà essere preso in automatico - al momento è hard coded

        // Le info dell'oggetto vanno chieste ad un apposito servizio

        cr_interfaces::msg::ObjectInfo object_info;
        object_info.id = current_object_id_;
        object_info.center.x = 0.500;
        object_info.center.y = 0.300;
        object_info.center.z = 0.425;
        object_info.size.x = 0.05; // [m]
        object_info.size.y = 0.05; // [m]
        object_info.size.z = 0.05; // [m]

        auto pick_goal_msg = Pick::Goal();
        pick_goal_msg.object_info = object_info;

        RCLCPP_INFO(this->get_logger(), "Sending pick goal...");

        auto send_pick_goal_options = rclcpp_action::Client<Pick>::SendGoalOptions();
        send_pick_goal_options.goal_response_callback = std::bind(&TaskOrchestrator::pick_goal_response_callback, this, _1);
        send_pick_goal_options.feedback_callback = std::bind(&TaskOrchestrator::pick_feedback_callback, this, _1, _2);
        send_pick_goal_options.result_callback = std::bind(&TaskOrchestrator::pick_result_callback, this, _1);
        this->pick_client_ptr_->async_send_goal(pick_goal_msg, send_pick_goal_options);
    }

    void TaskOrchestrator::pick_goal_response_callback(const GoalHandlePick::SharedPtr & goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Pick goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Pick goal accepted by server, waiting for result");
        }
    }

    void TaskOrchestrator::pick_feedback_callback(
        GoalHandlePick::SharedPtr, 
        const std::shared_ptr<const Pick::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "[Pick Feedback] Percentage: %.2f | Message: %s", feedback->percentage, feedback->feedback_msg.c_str());
    }

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
    void TaskOrchestrator::send_place_goal()
    {
        using namespace std::placeholders;

        if(!this->place_client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Place Action Server not available after waiting");
            // TODO: deve in qualche modo fallire
        }

        // TODO: tutto questo poi dovrà essere preso in automatico - al momento è hard coded
        cr_interfaces::msg::ObjectInfo object_info;
        object_info.id = current_object_id_;
        object_info.center.x = 0.899;
        object_info.center.y = 0.625;
        object_info.center.z = 0.939;
        object_info.size.x = 0.05; // [m]
        object_info.size.y = 0.05; // [m]
        object_info.size.z = 0.05; // [m]

        auto place_goal_msg = Place::Goal();
        place_goal_msg.object_info = object_info;
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

    void TaskOrchestrator::place_goal_response_callback(const GoalHandlePlace::SharedPtr & goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Place goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Place goal accepted by server, waiting for result");
        }
    }

    void TaskOrchestrator::place_feedback_callback(
        GoalHandlePlace::SharedPtr, 
        const std::shared_ptr<const Place::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "[Place Feedback] Percentage: %.2f | Message: %s", feedback->percentage, feedback->feedback_msg.c_str());
    }

    void TaskOrchestrator::place_result_callback(const GoalHandlePlace::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_ERROR(this->get_logger(), "Place goal succeeded!");
                is_busy_ = false;
                return;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Place goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Place goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
          }
    }

} // namespace task_orchestation
} // namespace cr