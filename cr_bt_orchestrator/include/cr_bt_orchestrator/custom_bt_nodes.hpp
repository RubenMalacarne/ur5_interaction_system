#ifndef CR_BT_ORCHESTRATOR__CUSTOM_BT_NODES_HPP_
#define CR_BT_ORCHESTRATOR__CUSTOM_BT_NODES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <cr_interfaces/msg/object_info.hpp>
#include <cr_interfaces/srv/get_object_info.hpp>
#include <cr_interfaces/srv/freeze_scene.hpp>
#include <cr_interfaces/action/pick.hpp>
#include <cr_interfaces/action/place.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp> // Per geometry_msgs::msg::Point
#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>

namespace cr
{
	namespace bt_nodes
	{
#define BT_ACTION_LOG_INFO(fmt, ...) RCLCPP_INFO(logger(), "[%s] " fmt, this->name().c_str(), ##__VA_ARGS__)
#define BT_ACTION_LOG_WARN(fmt, ...) RCLCPP_WARN(logger(), "[%s] " fmt, this->name().c_str(), ##__VA_ARGS__)
#define BT_ACTION_LOG_ERROR(fmt, ...) RCLCPP_ERROR(logger(), "[%s] " fmt, this->name().c_str(), ##__VA_ARGS__)

		// ------------------------------------------------------------------------------------------------------------------
		//                                       Service Wrapper - GET OBJECT INFO
		// ------------------------------------------------------------------------------------------------------------------
		class GetObjectInfo : public BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>
		{
		public:
			GetObjectInfo(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params);

			static BT::PortsList providedPorts();

			bool setRequest(typename cr_interfaces::srv::GetObjectInfo::Request::SharedPtr &request) override;
			BT::NodeStatus onResponseReceived(const typename cr_interfaces::srv::GetObjectInfo::Response::SharedPtr &response) override;

		protected:
			rclcpp::Logger logger() { return rclcpp::get_logger(this->name()); }
		};

		// ------------------------------------------------------------------------------------------------------------------
		//                                       Service Wrapper - FREEZE SCENE
		// ------------------------------------------------------------------------------------------------------------------
		class FreezeScene : public BT::RosServiceNode<cr_interfaces::srv::FreezeScene>
		{
		public:
			FreezeScene(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params);

			bool setRequest(typename Request::SharedPtr &request) override;
			BT::NodeStatus onResponseReceived(const typename Response::SharedPtr &response) override;

		protected:
			rclcpp::Logger logger() { return rclcpp::get_logger(this->name()); }
		};

		// ------------------------------------------------------------------------------------------------------------------
		//                                       Condition Node - IsAreaSafe
		// ------------------------------------------------------------------------------------------------------------------
		class IsAreaSafe : public BT::ConditionNode
		{
		public:
			IsAreaSafe(const std::string &name, const BT::NodeConfig &config);

			static BT::PortsList providedPorts();

			BT::NodeStatus tick() override;

			void updateSafetyStatus(bool is_safe);

		private:
			rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
			std::atomic_bool human_near_{false};
		};

		// ------------------------------------------------------------------------------------------------------------------
		//                                       Attesa di Area Safe
		// ------------------------------------------------------------------------------------------------------------------
		class EnsureAreaIsSafe : public BT::CoroActionNode {
		public:
			EnsureAreaIsSafe(const std::string& name, const BT::NodeConfig& config);

			static BT::PortsList providedPorts() { return {}; }

			BT::NodeStatus tick() override;
			void halt() override; // Importante per CoroActionNode

		private:
			std::atomic<bool> area_is_currently_safe_{false}; // Inizializza come preferisci
			rclcpp::Node::SharedPtr node_;
			rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_subscription_;
			bool first_tick_unsafe_logged_ = false;

			void safetyCallback(const std_msgs::msg::Bool::SharedPtr msg);
		};

		// ------------------------------------------------------------------------------------------------------------------
		//                                       Utility Node - LOG MESSAGE
		// ------------------------------------------------------------------------------------------------------------------
		class LogMessage : public BT::SyncActionNode
		{
		public:
			LogMessage(const std::string &name, const BT::NodeConfiguration &config);

			static BT::PortsList providedPorts();

			BT::NodeStatus tick() override;
		};
	} // namespace bt_nodes
} // namespace cr

#endif // CR_BT_ORCHESTRATOR__CUSTOM_BT_NODES_HPP_
