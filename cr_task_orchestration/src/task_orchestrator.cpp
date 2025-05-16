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

        this->get_object_info_client_ = this->create_client<cr_interfaces::srv::GetObjectInfo>("cr/get_object_info");

        this->freeze_scene_pub_ = this->create_publisher<cr_interfaces::msg::FreezeScene>("cr/freeze_scene", 10);

        this->execute_workflow_server_ptr_ = rclcpp_action::create_server<cr_interfaces::action::ExecuteWorkflow>(
            this,
            "cr/execute_workflow",
            std::bind(&TaskOrchestrator::handle_goal, this, _1, _2),
            std::bind(&TaskOrchestrator::handle_cancel, this, _1),
            std::bind(&TaskOrchestrator::handle_accepted, this, _1)
        );

        this->sub_command_ = this->create_subscription<std_msgs::msg::String>(
            "cr/pause_command",
            10,
            std::bind(&TaskOrchestrator::command_callback, this, _1)
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
            
            // Avvisiamo che la scena deve essere freezata
            cr_interfaces::msg::FreezeScene msg;
            msg.freeze = true;
            freeze_scene_pub_->publish(msg);

            is_busy_ = true;
            RCLCPP_INFO(this->get_logger(), "Goal accepted.");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
    }

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

    void  TaskOrchestrator::handle_accepted(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
    {
        using namespace std::placeholders;
        this->current_goal_handle_ = goal_handle;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread([this, goal_handle]() {
            this->get_object_info(goal_handle);
        }).detach();


    }

    //////////////////////////////////////////////////////
    //                  OBJECT_INFO                     //
    //////////////////////////////////////////////////////
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

    void TaskOrchestrator::get_object_info(int object_id)
    {
      auto request = std::make_shared<cr_interfaces::srv::GetObjectInfo::Request>();
      request->id = object_id;

      auto future = get_object_info_client_->async_send_request(
        request,
        [this](rclcpp::Client<cr_interfaces::srv::GetObjectInfo>::SharedFuture result_future)
        {
          if (result_future.get()->success)
          {
            this->target_object_ = result_future.get()->object_info;
            RCLCPP_INFO(this->get_logger(), "✅ Info oggetto ricevute – invio Pick");
            this->send_pick_goal();
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "❌ Info oggetto non trovate – task saltato");

            cr_interfaces::msg::FreezeScene msg;
            msg.freeze = false;
            freeze_scene_pub_->publish(msg);

            is_busy_ = false;

            // Prosegui con il prossimo task se non in pausa
            if (!is_paused_)
              run_next_task();
          }
        });
    }


    //////////////////////////////////////////////////////
    //                      COMMAND                     //
    //////////////////////////////////////////////////////
    void TaskOrchestrator::set_task_list(const std::vector<int> &tasks) {     
      task_list_            = tasks;
      current_task_index_   = 0;
      is_paused_            = false;
      RCLCPP_INFO(get_logger(), "Caricata task-list di %zu elementi", task_list_.size());
    }
    
    void TaskOrchestrator::command_callback(const std_msgs::msg::String::SharedPtr msg)
      {
        const auto &cmd = msg->data;
        if (cmd == "pause") {
          if (!is_paused_) {
            RCLCPP_INFO(get_logger(), "⏸️  Comando PAUSA ricevuto");
            is_paused_ = true;

            if (current_pick_goal_handle_) {
              pick_client_ptr_->async_cancel_goal(current_pick_goal_handle_.value());
              RCLCPP_INFO(get_logger(), "Richiesta cancellazione Pick in corso");
            }
            if (current_place_goal_handle_) {
              place_client_ptr_->async_cancel_goal(current_place_goal_handle_.value());
              RCLCPP_INFO(get_logger(), "Richiesta cancellazione Place in corso");
            }
          }
        } else if (cmd == "resume") {
          if (is_paused_) {
            RCLCPP_INFO(get_logger(), "▶️  Comando RIPRENDI ricevuto");
            is_paused_ = false;

            /* riprendi dallo stage in cui eravamo */
            if (current_stage_ == Stage::PICK) {
              send_pick_goal();
            } else if (current_stage_ == Stage::PLACE) {
              send_place_goal();
            } else {
              run_next_task(); 
            }
          }
        }
      }                                      

    
      void TaskOrchestrator::run_next_task()
    {
      if (is_paused_) return;
      if (current_task_index_ >= task_list_.size()) {
        RCLCPP_INFO(get_logger(), "🎉 Sequenza interna completata.");
        return;
      }
      int object_id = task_list_[current_task_index_];
      RCLCPP_INFO(get_logger(), "------ [Task %zu] Avvio workflow interno per oggetto id=%d ------",
                  current_task_index_, object_id);

      get_object_info(object_id); 
      ++current_task_index_;
    }
    //////////////////////////////////////////////////////
    //                      PICK                        //
    //////////////////////////////////////////////////////
    
    void TaskOrchestrator::send_pick_goal()
    {
        using namespace std::placeholders;
        if (is_paused_) return;                                
        current_stage_ = Stage::PICK;    

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
        RCLCPP_INFO(get_logger(), "🚀 Invio goal Pick");
        this->pick_client_ptr_->async_send_goal(pick_goal_msg, send_pick_goal_options);
    }

    void TaskOrchestrator::pick_goal_response_callback(const GoalHandlePick::SharedPtr & goal_handle)
    {
      current_pick_goal_handle_ = goal_handle;                          
      if (!goal_handle) RCLCPP_ERROR(get_logger(), "Goal Pick rifiutato");
      else     RCLCPP_INFO (get_logger(), "Goal Pick accettato");
    }

    void TaskOrchestrator::pick_feedback_callback(
        GoalHandlePick::SharedPtr, 
        const std::shared_ptr<const Pick::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "[Pick Feedback] Percentage: %.2f | Message: %s", feedback->percentage, feedback->feedback_msg.c_str());
    }

    void TaskOrchestrator::pick_result_callback(const GoalHandlePick::WrappedResult & result)
    {
        current_pick_goal_handle_.reset();      
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
        if (is_paused_) return;                                
        current_stage_ = Stage::PLACE;        
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

    void TaskOrchestrator::place_goal_response_callback(const GoalHandlePlace::SharedPtr & goal_handle)
    {
        current_place_goal_handle_ = goal_handle;                       
        if (!goal_handle) RCLCPP_ERROR(get_logger(), "Goal Place rifiutato");
        else     RCLCPP_INFO (get_logger(), "Goal Place accettato");
    }

    void TaskOrchestrator::place_feedback_callback(
        GoalHandlePlace::SharedPtr, 
        const std::shared_ptr<const Place::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "[Place Feedback] Percentage: %.2f | Message: %s", feedback->percentage, feedback->feedback_msg.c_str());
    }

    void TaskOrchestrator::place_result_callback(const GoalHandlePlace::WrappedResult & result)
    {
        current_place_goal_handle_.reset();
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
                RCLCPP_ERROR(this->get_logger(), "Place goal succeeded!");
                
                cr_interfaces::msg::FreezeScene msg;
                msg.freeze = false;
                freeze_scene_pub_->publish(msg);
            
                is_busy_ = false;
                current_stage_  = Stage::NONE;
            
                // if (current_goal_handle_) {
                //     auto result = std::make_shared<ExecuteWorkflow::Result>();
                //     result->success = true;
                //     result->msg = "Workflow completed successfully";
                //     current_goal_handle_->succeed(result);
                //     current_goal_handle_.reset();
                // }
                if (!is_paused_) run_next_task();
            
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