#include "cr_bt_orchestrator/custom_bt_nodes.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace cr
{
    namespace bt_nodes
    {
        // ------------------------------------------------------------------------------------------------------------------
        //                                       Service Wrapper - GET OBJECT INFO
        // ------------------------------------------------------------------------------------------------------------------
        GetObjectInfo::GetObjectInfo(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
            : BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>(instance_name, conf, params) {}

        BT::PortsList GetObjectInfo::providedPorts()
        {
            return BT::RosServiceNode<cr_interfaces::srv::GetObjectInfo>::providedBasicPorts({
                BT::InputPort<std::string>("object_id"),
                BT::OutputPort<double>("object_center_x"),
                BT::OutputPort<double>("object_center_y"),
                BT::OutputPort<double>("object_center_z"),
                BT::OutputPort<double>("object_size_x"),
                BT::OutputPort<double>("object_size_y"),
                BT::OutputPort<double>("object_size_z")
            });
        }

        bool GetObjectInfo::setRequest(typename cr_interfaces::srv::GetObjectInfo::Request::SharedPtr &request)
        {
            std::string object_id_str;
            if (!getInput("object_id", object_id_str))
            {
                RCLCPP_ERROR(logger(), "Missing input [object_id]");
                return false;
            }

            try
            {
                int tmp = std::stoi(object_id_str);
                if (tmp < 0 || tmp > std::numeric_limits<uint8_t>::max())
                {
                    RCLCPP_ERROR(logger(),
                                 "object_id out of range [0..%u]",
                                 static_cast<unsigned>(std::numeric_limits<uint8_t>::max()));
                    return false;
                }
                request->id = static_cast<uint8_t>(tmp);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(logger(),
                             "Invalid object_id '%s': %s",
                             object_id_str.c_str(), e.what());
                return false;
            }
            return true;
        }

        BT::NodeStatus GetObjectInfo::onResponseReceived(const typename cr_interfaces::srv::GetObjectInfo::Response::SharedPtr &response)
        {
            if (!response)
            {
                RCLCPP_ERROR(logger(), "Service call failed (empty response)");
                return BT::NodeStatus::FAILURE;
            }
            if (!response->success)
            {
                RCLCPP_ERROR(logger(), "Service returned failure");
                return BT::NodeStatus::FAILURE;
            }
            setOutput("object_center_x", response->object_info.center.x);
            setOutput("object_center_y", response->object_info.center.y);
            setOutput("object_center_z", response->object_info.center.z);
            setOutput("object_size_x", response->object_info.size.x);
            setOutput("object_size_y", response->object_info.size.y);
            setOutput("object_size_z", response->object_info.size.z);
            RCLCPP_INFO(logger(),
                        "Received object info: center=[%.2f, %.2f, %.2f]",
                        response->object_info.center.x,
                        response->object_info.center.y,
                        response->object_info.center.z);
            return BT::NodeStatus::SUCCESS;
        }

        // ------------------------------------------------------------------------------------------------------------------
        //                                       Service Wrapper - FREEZE SCENE
        // ------------------------------------------------------------------------------------------------------------------
        FreezeScene::FreezeScene(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
            : BT::RosServiceNode<cr_interfaces::srv::FreezeScene>(instance_name, conf, params) {}

        bool FreezeScene::setRequest(typename Request::SharedPtr& request)
        {
            request->freeze = true;
            return true;
        }

        BT::NodeStatus FreezeScene::onResponseReceived(const typename Response::SharedPtr &response)
        {
            if (!response)
            {
                RCLCPP_ERROR(logger(), "Service call failed (empty response)");
                return BT::NodeStatus::FAILURE;
            }
            if (!response->success)
            {
                RCLCPP_ERROR(logger(), "Service returned failure");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(logger(), "The planning scene was successfully frozen.");
            return BT::NodeStatus::SUCCESS;
        }

        // ------------------------------------------------------------------------------------------------------------------
        //                                       Action Wrapper - EXECUTE PICK
        // ------------------------------------------------------------------------------------------------------------------
        ExecutePick::ExecutePick(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
            : BT::RosActionNode<cr_interfaces::action::Pick>(instance_name, conf, params) {}

        BT::PortsList ExecutePick::providedPorts()
        {
            return BT::RosActionNode<cr_interfaces::action::Pick>::providedBasicPorts({BT::InputPort<cr_interfaces::msg::ObjectInfo>("object_info")});
        }

        bool ExecutePick::setGoal(Goal &goal)
        {
            cr_interfaces::msg::ObjectInfo obj;
            if (!getInput<cr_interfaces::msg::ObjectInfo>("object_info", obj))
            {
                BT_ACTION_LOG_ERROR("missing required input [object_info]");
                return false;
            }
            goal.object_info = obj;
            BT_ACTION_LOG_INFO("goal populated");
            return true;
        }

        BT::NodeStatus ExecutePick::onFailure(BT::ActionNodeErrorCode error)
        {
            switch (error)
            {
            case BT::ActionNodeErrorCode::SERVER_UNREACHABLE:
                BT_ACTION_LOG_ERROR("server unreachable");
                break;
            case BT::ActionNodeErrorCode::SEND_GOAL_TIMEOUT:
                BT_ACTION_LOG_ERROR("send-goal timeout");
                break;
            case BT::ActionNodeErrorCode::GOAL_REJECTED_BY_SERVER:
                BT_ACTION_LOG_ERROR("goal rejected by server");
                break;
            case BT::ActionNodeErrorCode::ACTION_ABORTED:
                BT_ACTION_LOG_ERROR("action aborted by server");
                break;
            case BT::ActionNodeErrorCode::ACTION_CANCELLED:
                BT_ACTION_LOG_WARN("action cancelled");
                break;
            case BT::ActionNodeErrorCode::INVALID_GOAL:
                BT_ACTION_LOG_ERROR("invalid goal");
                break;
            default:
                BT_ACTION_LOG_ERROR("unknown error code [%d]", static_cast<int>(error));
                break;
            }
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus ExecutePick::onFeedback(const std::shared_ptr<const Feedback> fb)
        {
            float pct = fb->percentage;
            BT_ACTION_LOG_INFO("progress %.1f%%", pct);
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus ExecutePick::onResultReceived(const WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                if (result.result->success)
                {
                    BT_ACTION_LOG_INFO("succeeded");
                    return BT::NodeStatus::SUCCESS;
                }
                BT_ACTION_LOG_WARN("completed with failure");
                return BT::NodeStatus::FAILURE;

            case rclcpp_action::ResultCode::ABORTED:
                BT_ACTION_LOG_ERROR("aborted by server");
                return BT::NodeStatus::FAILURE;

            case rclcpp_action::ResultCode::CANCELED:
                BT_ACTION_LOG_WARN("cancelled");
                return BT::NodeStatus::FAILURE;

            default:
                BT_ACTION_LOG_ERROR("unknown result code");
                return BT::NodeStatus::FAILURE;
            }
        }

        // ------------------------------------------------------------------------------------------------------------------
        //                                       Action Wrapper - EXECUTE PLACE
        // ------------------------------------------------------------------------------------------------------------------
        ExecutePlace::ExecutePlace(const std::string &instance_name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
            : BT::RosActionNode<cr_interfaces::action::Place>(instance_name, conf, params) {}

        BT::PortsList ExecutePlace::providedPorts()
        {
            return BT::RosActionNode<cr_interfaces::action::Place>::providedBasicPorts({BT::InputPort<cr_interfaces::msg::ObjectInfo>("object_info"),
                                                                                        BT::InputPort<geometry_msgs::msg::Point>("target_position")});
        }

        bool ExecutePlace::setGoal(Goal &goal)
        {
            cr_interfaces::msg::ObjectInfo obj;
            geometry_msgs::msg::Point pt;
            if (!getInput<cr_interfaces::msg::ObjectInfo>("object_info", obj))
            {
                BT_ACTION_LOG_ERROR("missing required input [object_info]");
                return false;
            }
            if (!getInput<geometry_msgs::msg::Point>("target_position", pt))
            {
                BT_ACTION_LOG_ERROR("missing required input [target_position]");
                return false;
            }
            goal.object_info = obj;
            goal.target_position = pt;
            BT_ACTION_LOG_INFO("goal populated (target: %.2f, %.2f, %.2f)", pt.x, pt.y, pt.z);
            return true;
        }

        BT::NodeStatus ExecutePlace::onFailure(BT::ActionNodeErrorCode error)
        {
            switch (error)
            {
            case BT::ActionNodeErrorCode::SERVER_UNREACHABLE:
                BT_ACTION_LOG_ERROR("server unreachable");
                break;
            case BT::ActionNodeErrorCode::SEND_GOAL_TIMEOUT:
                BT_ACTION_LOG_ERROR("send-goal timeout");
                break;
            case BT::ActionNodeErrorCode::GOAL_REJECTED_BY_SERVER:
                BT_ACTION_LOG_ERROR("goal rejected by server");
                break;
            case BT::ActionNodeErrorCode::ACTION_ABORTED:
                BT_ACTION_LOG_ERROR("action aborted by server");
                break;
            case BT::ActionNodeErrorCode::ACTION_CANCELLED:
                BT_ACTION_LOG_WARN("action cancelled");
                break;
            case BT::ActionNodeErrorCode::INVALID_GOAL:
                BT_ACTION_LOG_ERROR("invalid goal");
                break;
            default:
                BT_ACTION_LOG_ERROR("unknown error code [%d]", static_cast<int>(error));
                break;
            }
            return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus ExecutePlace::onFeedback(const std::shared_ptr<const Feedback> fb)
        {
            float pct = fb->percentage;
            BT_ACTION_LOG_INFO("progress %.1f%%", pct);
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus ExecutePlace::onResultReceived(const WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                if (result.result->success)
                {
                    BT_ACTION_LOG_INFO("succeeded: %s", result.result->result_msg.c_str());
                    return BT::NodeStatus::SUCCESS;
                }
                BT_ACTION_LOG_WARN("completed with failure: %s", result.result->result_msg.c_str());
                return BT::NodeStatus::FAILURE;

            case rclcpp_action::ResultCode::ABORTED:
                BT_ACTION_LOG_ERROR("aborted by server");
                return BT::NodeStatus::FAILURE;

            case rclcpp_action::ResultCode::CANCELED:
                BT_ACTION_LOG_WARN("cancelled");
                return BT::NodeStatus::FAILURE;

            default:
                BT_ACTION_LOG_ERROR("unknown result code");
                return BT::NodeStatus::FAILURE;
            }
        }

        // ------------------------------------------------------------------------------------------------------------------
        //                                       Utility Node - LOG MESSAGE
        // ------------------------------------------------------------------------------------------------------------------
        LogMessage::LogMessage(const std::string &name, const BT::NodeConfiguration &config)
            : BT::SyncActionNode(name, config) {}

        BT::PortsList LogMessage::providedPorts()
        {
            return {BT::InputPort<std::string>("message")};
        }

        BT::NodeStatus LogMessage::tick()
        {
            auto msg = getInput<std::string>("message");
            if (!msg)
            {
                RCLCPP_ERROR(rclcpp::get_logger("LogMessageNode"), "Missing port [message]");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(rclcpp::get_logger("LogMessageNode"), "BT_LOG: %s",
                        msg.value().c_str());
            return BT::NodeStatus::SUCCESS;
        }

    } // namespace bt_nodes
} // namespace cr
