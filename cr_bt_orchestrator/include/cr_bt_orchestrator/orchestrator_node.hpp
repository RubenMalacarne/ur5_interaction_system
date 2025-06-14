/**
 * @file orchestrator_node.hpp
 * @brief Definition of the OrchestratorNode class responsible for managing the entire pick-and-place workflow using a Behavior Tree.
 *
 * This node initializes and runs a Behavior Tree engine that coordinates the robot’s workflow based on action requests
 * and real-time conditions (pause and stop) or unsafe area detection.
 */

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

    /**
     * @class OrchestratorNode
     * @brief Main node that manages the execution of the pick-and-place workflow using Behavior Trees.
     *
     * This node exposes an action interface to receive external requests, initializes the BT factory,
     * and launches a tree execution in a separate thread. It ensures safety and pause mechanisms
     * are respected by checking runtime conditions from ROS topics.
     */
    class OrchestratorNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor. Loads parameters and sets up the BT execution environment.
         * @param options Node options for initialization.
         */
        explicit OrchestratorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        /**
         * @brief Default destructor.
         *
         */
        ~OrchestratorNode() override = default;

    private:
        // Type aliases for action
        using ExecuteWorkflow = cr_interfaces::action::ExecuteWorkflow;
        using GoalHandleExecuteWorkflow = rclcpp_action::ServerGoalHandle<ExecuteWorkflow>;

        /// Action server for external workflow execution requests.
        rclcpp_action::Server<ExecuteWorkflow>::SharedPtr execute_workflow_server_;

        /// Publisher to send GUI log messages to a centralized UI/logging system.
        rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr gui_log_pub_;

        /**
         * @brief Callback executed upon receiving a new goal request.
         * @param uuid Unique ID of the goal.
         * @param goal Pointer to the goal message.
         * @return The decision to accept or reject the goal.
         */
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ExecuteWorkflow::Goal> goal);

        /**
         * @brief Callback executed when a cancellation is requested.
         * @param goal_handle Handle to the goal to cancel.
         * @return The cancellation response.
         */
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle);

        /**
         * @brief Main execution method that runs the BT asynchronously in a new thread.
         * @param goal_handle Handle to the goal being executed.
         */
        void execute(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle);

        /**
         * @brief Loads BT-related configuration parameters into the tree's blackboard.
         * @param blackboard Pointer to the blackboard used in the current BT instance.
         */
        void loadConfigurationToBlackboard(BT::Blackboard::Ptr blackboard);

        /**
         * @brief Sends a log message to the GUI indicating the node is idle.
         */
        void publishWaitingMessage();

        /// BT factory used to register and instantiate nodes and subtrees.
        BT::BehaviorTreeFactory factory_;

        /// Internal helper to configure the factory and register all BT nodes.
        void setupBTFactory();

        /// Timer used to delay BT factory setup until node construction is complete.
        rclcpp::TimerBase::SharedPtr setup_timer_;

        /// Flag indicating whether the BT factory has been initialized.
        bool bt_factory_initialized_{false};

        /// Flag used to track whether a BT execution is currently running.
        std::atomic_bool bt_running_{false};

        // --------------------------------------------------------------------------------
        // Parameters exposed to the blackboard and used across the BT workflow execution
        // --------------------------------------------------------------------------------

        /// Distance used for positioning before approaching an object.
        double pre_approach_distance_;

        /// Final approach distance toward the object.
        double approach_distance_;

        /// Gripper joint value for the open state.
        double gripper_open_value_;

        /// Gripper joint value for the closed state.
        double gripper_close_value_;

        /// Z-axis position for the "home" pose.
        double home_z_position_;

        /// X offset applied during the placement phase.
        double place_offset_x_;

        /// Z offset applied during the placement phase.
        double place_offset_z_;
    };

} // namespace cr::bt::orchestrator

#endif // CR_BT_ORCHESTRATOR__ORCHESTRATOR_NODE_HPP_
