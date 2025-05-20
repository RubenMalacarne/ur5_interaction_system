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
		//                                       Action Wrapper - EXECUTE PICK
		// ------------------------------------------------------------------------------------------------------------------
		class ExecutePick : public BT::RosActionNode<cr_interfaces::action::Pick>
		{
		public:
			ExecutePick(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params);

			static BT::PortsList providedPorts();

			bool setGoal(Goal &goal) override;
			BT::NodeStatus onResultReceived(const WrappedResult &result) override;
			BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
			BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

		protected:
			rclcpp::Logger logger() { return rclcpp::get_logger(this->name()); }
		};

		// ------------------------------------------------------------------------------------------------------------------
		//                                       Action Wrapper - EXECUTE PLACE
		// ------------------------------------------------------------------------------------------------------------------
		class ExecutePlace : public BT::RosActionNode<cr_interfaces::action::Place>
		{
		public:
			ExecutePlace(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params);

			static BT::PortsList providedPorts();

			bool setGoal(Goal &goal) override;
			BT::NodeStatus onResultReceived(const WrappedResult &result) override;
			BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
			BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

		protected:
			rclcpp::Logger logger() { return rclcpp::get_logger(this->name()); }
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

		            // ------------------------------------------------------------------------------------------------------------------
            //                                       Utility Node - COMPUTE PRE-APPROACH Z
            // ------------------------------------------------------------------------------------------------------------------
            class ComputePreApproachZ : public BT::SyncActionNode
            {
            public:
                ComputePreApproachZ(const std::string& name, const BT::NodeConfiguration& config);

                static BT::PortsList providedPorts();

                BT::NodeStatus tick() override;

            private:
                rclcpp::Node::SharedPtr nh_; // Per il logging
            };

            // ------------------------------------------------------------------------------------------------------------------
            //                                       Utility Node - COMPUTE OBJECT CENTER XY
            // ------------------------------------------------------------------------------------------------------------------
            class ComputeObjectCenterXY : public BT::SyncActionNode
            {
            public:
                ComputeObjectCenterXY(const std::string& name, const BT::NodeConfiguration& config);

                static BT::PortsList providedPorts();

                BT::NodeStatus tick() override;

            private:
                rclcpp::Node::SharedPtr nh_; // Per il logging
            };

            // ------------------------------------------------------------------------------------------------------------------
            //                                       Utility Node - COMPUTE APPROACH Z
            // ------------------------------------------------------------------------------------------------------------------
            class ComputeApproachZ : public BT::SyncActionNode
            {
            public:
                ComputeApproachZ(const std::string& name, const BT::NodeConfiguration& config);

                static BT::PortsList providedPorts();

                BT::NodeStatus tick() override;

            private:
                rclcpp::Node::SharedPtr nh_; // Per il logging
            };

	} // namespace bt_nodes
} // namespace cr

#endif // CR_BT_ORCHESTRATOR__CUSTOM_BT_NODES_HPP_
