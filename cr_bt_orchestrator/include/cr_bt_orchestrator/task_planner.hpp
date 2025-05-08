#ifndef CR_BT_ORCHESTRATOR_TASK_PLANNER_HPP_
#define CR_BT_ORCHESTRATOR_TASK_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cr_interfaces/action/execute_workflow.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>

namespace cr
{
    namespace bt_orchestrator
    {

        class BtOrchestratorNode : public rclcpp::Node
        {
        public:
            /** Costruttore */
            explicit BtOrchestratorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        private:
            /** Tutto il necessario per l'implementazione della parte ation server per ExecuteWorkflow */
            using ExecuteWorkflow = cr_interfaces::action::ExecuteWorkflow;
            using GoalHandleExecuteWorkflow = rclcpp_action::ServerGoalHandle<ExecuteWorkflow>;
            rclcpp_action::Server<ExecuteWorkflow>::SharedPtr execute_workflow_server_;
            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ExecuteWorkflow::Goal> goal);
            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle);
            /** Metodo dedicato al tick del tree e esecuzione del workflow */
            void execute(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle);

            /** Tutto il necessario per l'implementazione del BT */
            BT::BehaviorTreeFactory factory_;
            BT::Tree tree_;
            BT::Blackboard::Ptr blackboard_; 
            std::unique_ptr<BT::StdCoutLogger> stdout_logger_;
            std::unique_ptr<BT::FileLogger2> groot_logger_;
        };

    } // namespace bt_orchestrator
} // namespace cr

#endif // CR_BT_ORCHESTRATOR_TASK_PLANNER_HPP_
