/**
 * @file nodes.hpp
 * @brief Behavior Tree nodes designed to support and coordinate the execution flow defined within the system.
 *
 * This header defines all Behavior Tree nodes used in the orchestrator workflow. These nodes
 * are responsible for managing scene-level operations, safety conditions, user interactions,
 * and global control flow. They interface with services, topics, and blackboard elements to
 * coordinate the execution of pick-and-place tasks in a safe and modular manner.
 */

#ifndef CR_BT_ORCHESTRATOR__NODES_HPP_
#define CR_BT_ORCHESTRATOR__NODES_HPP_

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <cr_interfaces/msg/object_info.hpp>
#include <cr_interfaces/srv/get_object_info.hpp>
#include <cr_interfaces/srv/freeze_scene.hpp>
#include <cr_interfaces/msg/log.hpp>
#include <std_msgs/msg/bool.hpp>

namespace cr::bt::orchestrator::nodes
{

#define BT_ACTION_LOG_INFO(fmt, ...) RCLCPP_INFO(logger(), "[%s] " fmt, this->name().c_str(), ##__VA_ARGS__)
#define BT_ACTION_LOG_WARN(fmt, ...) RCLCPP_WARN(logger(), "[%s] " fmt, this->name().c_str(), ##__VA_ARGS__)
#define BT_ACTION_LOG_ERROR(fmt, ...) RCLCPP_ERROR(logger(), "[%s] " fmt, this->name().c_str(), ##__VA_ARGS__)

	/**
	 * @class GetObjectInfo
	 * @brief Service node that retrieves full information about a scene object given its label.
	 *
	 * Uses the `cr_interfaces::srv::GetObjectInfo` service to query spatial and identification
	 * information about objects to be manipulated. Outputs include ID, center coordinates,
	 * and dimensions.
	 */
	class GetObjectInfo : public BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>
	{
	public:
		GetObjectInfo(const std::string &instance_name,
					  const BT::NodeConfig &conf,
					  const BT::RosNodeParams &params);

		static BT::PortsList providedPorts();

		bool setRequest(typename Request::SharedPtr &request) override;
		BT::NodeStatus onResponseReceived(const typename Response::SharedPtr &response) override;

	protected:
		rclcpp::Logger logger() const { return rclcpp::get_logger(this->name()); }
	};

	/**
	 * @class FreezeScene
	 * @brief Service node to freeze or unfreeze the planning scene.
	 *
	 * Calls the `cr_interfaces::srv::FreezeScene` service to lock or unlock
	 * the state of the environment. Used to stabilize planning operations.
	 */
	class FreezeScene : public BT::RosServiceNode<cr_interfaces::srv::FreezeScene>
	{
	public:
		FreezeScene(const std::string &instance_name,
					const BT::NodeConfig &conf,
					const BT::RosNodeParams &params);

		static BT::PortsList providedPorts();

		bool setRequest(typename Request::SharedPtr &request) override;
		BT::NodeStatus onResponseReceived(const typename Response::SharedPtr &response) override;

	protected:
		rclcpp::Logger logger() const { return rclcpp::get_logger(this->name()); }
	};

	/**
	 * @class IsAreaSafe
	 * @brief Condition node that checks if the workspace area is free of humans.
	 *
	 * Subscribes to a boolean topic indicating the presence of a human in the area.
	 * Returns SUCCESS if the area is safe, FAILURE otherwise.
	 */
	class IsAreaSafe : public BT::ConditionNode
	{
	public:
		IsAreaSafe(const std::string &name, const BT::NodeConfig &config);

		static BT::PortsList providedPorts();

		BT::NodeStatus tick() override;
		void updateSafetyStatus(bool is_safe);

	private:
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_subscription_;
		rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr gui_log_pub_;
		std::atomic<bool> human_near_{false};
	};

	/**
	 * @class IsStopRequested
	 * @brief Condition node that listens for external stop commands.
	 *
	 * Subscribes to a boolean stop topic and returns SUCCESS when a stop is requested,
	 * otherwise FAILURE. Allows interruption of the workflow.
	 */
	class IsStopRequested : public BT::ConditionNode
	{
	public:
		IsStopRequested(const std::string &name, const BT::NodeConfig &config);

		static BT::PortsList providedPorts();

		BT::NodeStatus tick() override;
		void updateStopRequestedStatus(bool is_safe);

	private:
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_command_sub_;
		rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr gui_log_pub_;
		std::atomic<bool> stop_requested_{false};
	};

	/**
	 * @class IsPauseRequested
	 * @brief Condition node that detects if the user has requested a pause.
	 *
	 * Subscribes to a pause topic. Returns SUCCESS when a pause is active, FAILURE otherwise.
	 * Helps coordinate safe pausing and resuming of operations.
	 */
	class IsPauseRequested : public BT::ConditionNode
	{
	public:
		IsPauseRequested(const std::string &name, const BT::NodeConfig &config);

		static BT::PortsList providedPorts();

		BT::NodeStatus tick() override;
		void updatePauseRequestedStatus(bool is_safe);

	private:
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_command_sub_;
		rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr gui_log_pub_;
		std::atomic<bool> pause_requested_{false};
	};

	/**
	 * @class WaitForTheGoAhead
	 * @brief CoroActionNode that blocks execution until safety and resume conditions are met.
	 *
	 * Combines area safety and resume request logic. Only returns SUCCESS when the area is safe
	 * and the user has explicitly requested to resume operations. Continuously publishes log updates.
	 */
	class WaitForTheGoAhead : public BT::CoroActionNode
	{
	public:
		WaitForTheGoAhead(const std::string &name, const BT::NodeConfig &config);

		static BT::PortsList providedPorts() { return {}; }

		BT::NodeStatus tick() override;
		void halt() override;

	private:
		void safetyCallback(const std_msgs::msg::Bool::SharedPtr msg);
		void pauseCallback(const std_msgs::msg::Bool::SharedPtr msg);

		rclcpp::Node::SharedPtr node_;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_subscription_;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_command_sub_;
		rclcpp::Publisher<cr_interfaces::msg::Log>::SharedPtr gui_log_pub_;

		std::atomic<bool> human_resume_requested_{true};
		std::atomic<bool> area_is_currently_safe_{false};

		bool first_tick_unsafe_logged_{false};
		bool first_tick_pause_logged_{false};
	};

} // namespace cr::bt::orchestrator::nodes

#endif // CR_BT_ORCHESTRATOR__NODES_HPP_
