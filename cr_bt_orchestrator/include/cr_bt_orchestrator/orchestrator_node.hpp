#ifndef CR_BT_ORCHESTRATOR__ORCHESTRATOR_NODE_HPP_
#define CR_BT_ORCHESTRATOR__ORCHESTRATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cr_interfaces/action/execute_workflow.hpp>
#include <cr_interfaces/msg/log.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>

#include <memory>
#include <chrono>
#include <string>
#include <thread>

namespace cr::bt::orchestrator
{
    class OrchestratorNode : public rclcpp::Node
    {
    public:
        explicit OrchestratorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        using ExecuteWorkflow = cr_interfaces::action::ExecuteWorkflow;
        using GoalHandleExecuteWorkflow = rclcpp_action::ServerGoalHandle<ExecuteWorkflow>;

        rclcpp_action::Server<ExecuteWorkflow>::SharedPtr execute_workflow_server_;
        rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr gui_log_pub_; // Questo pub è condiviso tra tutti coloro che devono utilizzarlo

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ExecuteWorkflow::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle);

        void execute(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle);

        void loadConfigurationToBlackboard(BT::Blackboard::Ptr blackboard);

        void publishWaitingMessage();

        BT::BehaviorTreeFactory factory_;
        void setupBTFactory();
        rclcpp::TimerBase::SharedPtr setup_timer_;
        bool bt_factory_initialized_{false};
        std::atomic_bool bt_running_{false};

        double pre_approach_distance_;
        double approach_distance_;
        double gripper_open_value_;
        double gripper_close_value_;
        double home_z_position_;
        double place_offset_x_;
        double place_offset_z_;
    };
} // namespace cr

#endif // CR_BT_ORCHESTRATOR__ORCHESTRATOR_NODE_HPP_
