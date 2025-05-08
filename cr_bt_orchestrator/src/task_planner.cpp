#include "cr_bt_orchestrator/task_planner.hpp"
#include "cr_bt_orchestrator/custom_bt_nodes.hpp"

namespace cr
{
    namespace bt_orchestrator
    {

        BtOrchestratorNode::BtOrchestratorNode(const rclcpp::NodeOptions &options)
            : Node("bt_orchestrator_node", options)
        {

            RCLCPP_INFO(this->get_logger(), "Initializing BT Orchestrator Node");

            // Creazione dell'action server per l'invio di request da parte di un client per l'esecuzione del workflow
            this->execute_workflow_server_ = rclcpp_action::create_server<ExecuteWorkflow>(
                this,
                "cr/execute_workflow",
                std::bind(&BtOrchestratorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&BtOrchestratorNode::handle_cancel, this, std::placeholders::_1),
                std::bind(&BtOrchestratorNode::execute, this, std::placeholders::_1));

            // Impostazione del BT
            factory_.registerNodeType<cr::bt_nodes::GetObjectInfo>("GetObjectInfo");
            factory_.registerNodeType<cr::bt_nodes::ExecutePick>("ExecutePick");
            factory_.registerNodeType<cr::bt_nodes::ExecutePlace>("ExecutePlace");
            factory_.registerNodeType<cr::bt_nodes::LogMessage>("LogSuccess");
        
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("cr_bt_orchestrator");
            std::string bt_xml_path = package_share_directory + "/bt_xml/simple_pick_place.xml";

            RCLCPP_INFO(this->get_logger(), "Loading BT from: %s", bt_xml_path.c_str());

            this->blackboard_ = BT::Blackboard::create();
            tree_ = factory_.createTreeFromFile(bt_xml_path, blackboard_);
        
            // Aggiunta logger
            // Logger su console
            stdout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
            // Logger per Groot2 (visualizzazione)
            std::string groot_log_path = package_share_directory + "/bt_trace.btlog";
            FILE *f = fopen(groot_log_path.c_str(), "w");
            if (f)
              fclose(f);
            groot_logger_ = std::make_unique<BT::FileLogger2>(tree_, groot_log_path);

            RCLCPP_INFO(this->get_logger(), "BT Orchestrator Node initialized. Ready to execute.");

        }

        rclcpp_action::GoalResponse BtOrchestratorNode::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ExecuteWorkflow::Goal> goal){
            // Al momento accettiamo subito
            RCLCPP_INFO(this->get_logger(), "Received goal request for object %d", goal->object_id);
            (void)uuid;
            // Accetta sempre il goal per questo esempio. In un sistema reale, potresti
            // voler fare dei controlli (es. se l'oggetto esiste).
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse BtOrchestratorNode::handle_cancel(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void BtOrchestratorNode::execute(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle){
            RCLCPP_INFO(this->get_logger(), "Executing goal");

            auto result   = std::make_shared<ExecuteWorkflow::Result>();
            auto feedback = std::make_shared<ExecuteWorkflow::Feedback>();
  
            // scrittura sul backboard
            blackboard_->set("object_id", std::to_string(goal_handle->get_goal()->object_id));

            // Tick dell'albero
            BT::NodeStatus status = BT::NodeStatus::RUNNING;
            while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
                if (goal_handle->is_canceling()) {
                  RCLCPP_INFO(this->get_logger(), "Goal canceled");
                  goal_handle->canceled(result);
                  return;
                }
            
                status = tree_.tickOnce();
                goal_handle->publish_feedback(feedback);
            
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

              // Conclusione con succeed o abort
            if (status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Workflow succeeded");
                goal_handle->succeed(result);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Workflow failed");
                goal_handle->abort(result);
            }                

        }

    } // namespace bt_orchestrator
} // namespace cr