#ifndef CR_BT_ORCHESTRATOR_TASK_PLANNER_HPP_
#define CR_BT_ORCHESTRATOR_TASK_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cr_interfaces/action/execute_workflow.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>

// BehaviorTree.CPP
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>

// ROS2 integration for BT.CPP
#include <behaviortree_ros2/ros_node_params.hpp>

#include "cr_bt_orchestrator/custom_bt_nodes.hpp"

#include <memory>
#include <chrono>
#include <string>

namespace cr
{
	namespace bt_orchestrator
	{

		class BtOrchestratorNode : public rclcpp::Node
		{
		public:
			explicit BtOrchestratorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

		private:
			// Action server per ExecuteWorkflow
			using ExecuteWorkflow = cr_interfaces::action::ExecuteWorkflow;
			using GoalHandleExecuteWorkflow = rclcpp_action::ServerGoalHandle<ExecuteWorkflow>;
			rclcpp_action::Server<ExecuteWorkflow>::SharedPtr execute_workflow_server_;
			rclcpp_action::GoalResponse handle_goal(
				const rclcpp_action::GoalUUID &uuid,
				std::shared_ptr<const ExecuteWorkflow::Goal> goal);
			rclcpp_action::CancelResponse handle_cancel(
				const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle);
			void execute(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle);

			// BehaviorTree
			BT::BehaviorTreeFactory factory_;
			BT::Tree tree_;
			BT::Blackboard::Ptr blackboard_;
			std::unique_ptr<BT::StdCoutLogger> stdout_logger_;
			std::unique_ptr<BT::FileLogger2> groot_logger_;

			// Setup post‐costruttore
			void setupBT();
			rclcpp::TimerBase::SharedPtr setup_timer_;
			bool bt_initialized_{false};

			// Logica di pause e resume
			rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pause_command_sub_;
		};

	} // namespace bt_orchestrator
} // namespace cr

#endif // CR_BT_ORCHESTRATOR_TASK_PLANNER_HPP_
