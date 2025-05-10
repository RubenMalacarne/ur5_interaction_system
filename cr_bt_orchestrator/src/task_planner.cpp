#include "cr_bt_orchestrator/task_planner.hpp"
#include <cr_bt_orchestrator/custom_bt_nodes.hpp>
using namespace std::chrono_literals;

namespace cr
{
    namespace bt_orchestrator
    {

        BtOrchestratorNode::BtOrchestratorNode(const rclcpp::NodeOptions &options)
            : Node("bt_orchestrator_node", options)
        {
            RCLCPP_INFO(get_logger(), "Initializing BT Orchestrator Node");
            pick_client_ptr_ = rclcpp_action::create_client<Pick>(this, "cr/pick_action");  
            // 1) Action server
            execute_workflow_server_ = rclcpp_action::create_server<ExecuteWorkflow>(
                this,
                "cr/execute_workflow",
                std::bind(&BtOrchestratorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&BtOrchestratorNode::handle_cancel, this, std::placeholders::_1),
                std::bind(&BtOrchestratorNode::execute, this, std::placeholders::_1));


            // 2) Pause behavior
            pause_sub_ = create_subscription<std_msgs::msg::Bool>(
                "cr/pause_robot",
                10,
                [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    if (msg->data)
                    {
                        RCLCPP_INFO(get_logger(), "Pause requested");
                        pause_requested_ = true;
                    }
                    else
                    {
                        RCLCPP_INFO(get_logger(), "Resume requested");
                        pause_requested_ = false;
                    }
                });
            //get_ob_inf
            get_object_info_client_ = this->create_client<cr_interfaces::srv::GetObjectInfo>("cr/get_object_info");
            
            //freeze scene: 
            freeze_scene_pub_ = this->create_publisher<cr_interfaces::msg::FreezeScene>("cr/freeze_scene", 10);

            // 2) Timer a 0ms per post‐costructor setup
            setup_timer_ = create_wall_timer(
                0ms,
                std::bind(&BtOrchestratorNode::setupBT, this));
        }

        void BtOrchestratorNode::setupBT()
        {
            if (bt_initialized_)
            {
                return;
            }
            bt_initialized_ = true;
            setup_timer_->cancel();

            // A) registra i nodi custom, incluso il servizio
            BT::RosNodeParams params;
            params.nh = shared_from_this();
            params.default_port_value = "cr/get_object_info";

            factory_.registerNodeType<cr::bt_nodes::GetObjectInfo>("GetObjectInfo", params);
            factory_.registerNodeType<cr::bt_nodes::ExecutePick>("ExecutePick");
            factory_.registerNodeType<cr::bt_nodes::ExecutePlace>("ExecutePlace");
            factory_.registerNodeType<cr::bt_nodes::CheckHumanPresence>("CheckHumanPresence");
            factory_.registerNodeType<cr::bt_nodes::PauseRobot>("PauseRobot");
            factory_.registerNodeType<cr::bt_nodes::LogMessage>("LogSuccess");

            // B) carica l’XML
            const auto pkg_share = ament_index_cpp::get_package_share_directory("cr_bt_orchestrator");
            const auto bt_xml = pkg_share + "/bt_xml/simple_pick_place.xml";
            RCLCPP_INFO(get_logger(), "Loading BT from: %s", bt_xml.c_str());

            blackboard_ = BT::Blackboard::create();
            // metti anche il nodo ROS a disposizione dei BT‐nodes
            blackboard_->set<rclcpp::Node::SharedPtr>("ros_node", shared_from_this());

            tree_ = factory_.createTreeFromFile(bt_xml, blackboard_);

            // C) logger
            stdout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
            const auto log_path = pkg_share + "/bt_trace.btlog";
            FILE *f = fopen(log_path.c_str(), "w");
            if (f)
            {
                fclose(f);
            }
            groot_logger_ = std::make_unique<BT::FileLogger2>(tree_, log_path);

            RCLCPP_INFO(get_logger(), "BT Orchestrator Node initialized. Ready to execute.");
        }

        rclcpp_action::GoalResponse BtOrchestratorNode::handle_goal(const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ExecuteWorkflow::Goal> goal)
        {
            RCLCPP_INFO(get_logger(), "Received goal request for object %d", goal->object_id);
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

        rclcpp_action::CancelResponse BtOrchestratorNode::handle_cancel(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Received request to cancel goal");
            (void)goal_handle;

            // Avvisiamo che la scena non deve più essere freezata
            cr_interfaces::msg::FreezeScene msg;
            msg.freeze = false;
            freeze_scene_pub_->publish(msg);

            is_busy_ = false;
            return rclcpp_action::CancelResponse::ACCEPT;
        }



        void BtOrchestratorNode::execute(
            const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Executing goal");

            auto result = std::make_shared<ExecuteWorkflow::Result>();
            auto feedback = std::make_shared<ExecuteWorkflow::Feedback>();

            // passaggio su blackboard (l’XML usa {target_object_id})
            blackboard_->set<std::string>(
                "object_id",
                std::to_string(goal_handle->get_goal()->object_id));

            // loop di tick
            BT::NodeStatus status = BT::NodeStatus::RUNNING;
            while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
            {
                if (goal_handle->is_canceling())
                {
                    RCLCPP_INFO(get_logger(), "Goal canceled");
                    goal_handle->canceled(result);
                    return;
                }
                if (pause_requested_)
                {
                    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000, "BT paused...");
                    std::this_thread::sleep_for(100ms);
                    continue;
                }
                status = tree_.tickOnce();
                goal_handle->publish_feedback(feedback);
                std::this_thread::sleep_for(10ms);
            }

            if (status == BT::NodeStatus::SUCCESS)
            {
                RCLCPP_INFO(get_logger(), "Workflow succeeded");
                goal_handle->succeed(result);
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Workflow failed");
                goal_handle->abort(result);
            }
            
            RCLCPP_INFO(get_logger(), "APPERTURA THREAD PER L-ESECUZIONE");
            std::thread{std::bind(&BtOrchestratorNode::get_object_info, this,goal_handle)}.detach();
        }

    //////////////////////////////////////////////////////
    //                  OBJECT_INFO                     //
    //////////////////////////////////////////////////////
    void BtOrchestratorNode::get_object_info(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle){
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
    void BtOrchestratorNode::send_pick_goal()
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
        send_pick_goal_options.goal_response_callback = std::bind(&BtOrchestratorNode::pick_goal_response_callback, this, _1);
        send_pick_goal_options.feedback_callback = std::bind(&BtOrchestratorNode::pick_feedback_callback, this, _1, _2);
        send_pick_goal_options.result_callback = std::bind(&BtOrchestratorNode::pick_result_callback, this, _1);
        this->pick_client_ptr_->async_send_goal(pick_goal_msg, send_pick_goal_options);
    }

    void BtOrchestratorNode::pick_goal_response_callback(const GoalHandlePick::SharedPtr & goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Pick goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Pick goal accepted by server, waiting for result");
            // TODO: Gestire il fatto che non è più busy e sbloccare la scena 
        }
    }

    void BtOrchestratorNode::pick_feedback_callback(
        GoalHandlePick::SharedPtr, 
        const std::shared_ptr<const Pick::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "[Pick Feedback] Percentage: %.2f | Message: %s", feedback->percentage, feedback->feedback_msg.c_str());
    }

    void BtOrchestratorNode::pick_result_callback(const GoalHandlePick::WrappedResult & result)
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

    void BtOrchestratorNode::send_place_goal()
    {
        RCLCPP_INFO(this->get_logger(), "Placeholder for send_place_goal");
        // TODO: Implement the logic for sending the place goal
    }
    } // namespace bt_orchestrator
} // namespace cr
