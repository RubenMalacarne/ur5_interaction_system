#include "cr_bt_orchestrator/orchestrator_node.hpp"
#include "cr_bt_orchestrator/nodes.hpp"

#include <cr_bt_common/log_message_node.hpp>
#include <cr_bt_common/gui_log_node.hpp>
#include <cr_bt_pick_place/bt_nodes_factory.hpp>
#include <cr_motion_core/motion_commander.hpp>

#include <behaviortree_cpp/xml_parsing.h>
#include <fstream>

#include <thread>

using namespace std::chrono_literals;

namespace cr::bt::orchestrator
{

    OrchestratorNode::OrchestratorNode(const rclcpp::NodeOptions &options)
        : Node("orchestrator", options)
    {
        RCLCPP_INFO(get_logger(), "Initializing BT Orchestrator Node...");

        pre_approach_distance_ = this->declare_parameter("pre_approach_distance", 0.15);
        approach_distance_ = this->declare_parameter("approach_distance", 0.06);
        gripper_open_value_ = this->declare_parameter("gripper_open_value", 0.0);
        gripper_close_value_ = this->declare_parameter("gripper_close_value", 0.8);
        home_z_position_ = this->declare_parameter("home_z_position", 1.25);
        place_offset_x_ = this->declare_parameter("place_offset_x", 0.1);
        place_offset_z_ = this->declare_parameter("place_offset_z", 0.1);

        RCLCPP_INFO(get_logger(), "Configuration loaded:");
        RCLCPP_INFO(get_logger(), "  pre_approach_distance: %.3f", pre_approach_distance_);
        RCLCPP_INFO(get_logger(), "  approach_distance: %.3f", approach_distance_);
        RCLCPP_INFO(get_logger(), "  gripper_open_value: %.3f", gripper_open_value_);
        RCLCPP_INFO(get_logger(), "  gripper_close_value: %.3f", gripper_close_value_);
        RCLCPP_INFO(get_logger(), "  home_z_position: %.3f", home_z_position_);
        RCLCPP_INFO(get_logger(), "  place_offset_x: %.3f", place_offset_x_);
        RCLCPP_INFO(get_logger(), "  place_offset_z: %.3f", place_offset_z_);

        auto gui_qos = rclcpp::QoS(10).transient_local();
        gui_log_pub_ = this->create_publisher<cr_interfaces::msg::Log>("cr/gui_log", gui_qos);

        execute_workflow_server_ = rclcpp_action::create_server<ExecuteWorkflow>(
            this,
            "cr/execute_workflow",
            std::bind(&OrchestratorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&OrchestratorNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&OrchestratorNode::execute, this, std::placeholders::_1));

        setup_timer_ = create_wall_timer(
            0ms,
            std::bind(&OrchestratorNode::setupBTFactory, this));
    }

    void OrchestratorNode::setupBTFactory()
    {
        if (bt_factory_initialized_)
        {
            return;
        }

        RCLCPP_INFO(get_logger(), "Setting up BehaviorTreeFactory...");

        BT::RosNodeParams default_ros_params;
        default_ros_params.nh = shared_from_this();

        BT::RosNodeParams freeze_params = default_ros_params;
        freeze_params.default_port_value = "cr/freeze_scene";
        factory_.registerNodeType<nodes::FreezeScene>("FreezeScene", freeze_params);

        BT::RosNodeParams get_object_info_params = default_ros_params;
        get_object_info_params.default_port_value = "cr/get_object_info";
        factory_.registerNodeType<nodes::GetObjectInfo>("GetObjectInfo", get_object_info_params);

        factory_.registerNodeType<nodes::IsStopRequested>("IsStopRequested");
        factory_.registerNodeType<nodes::IsPauseRequested>("IsPauseRequested");
        factory_.registerNodeType<nodes::IsAreaSafe>("IsAreaSafe");
        factory_.registerNodeType<nodes::WaitForTheGoAhead>("WaitForTheGoAhead");

        factory_.registerNodeType<cr::bt::common::GuiLog>("GuiLog");
        factory_.registerNodeType<cr::bt::common::LogMessageNode>("LogMessage");

        cr::bt::pick_place::registerNodes(factory_, shared_from_this());
        cr::bt::pick_place::registerSubtrees(factory_);

        try
        {
            const auto orchestrator_pkg = ament_index_cpp::get_package_share_directory("cr_bt_orchestrator");
            const auto main_xml = orchestrator_pkg + "/bt_xml/pick_place_workflow.xml";
            factory_.registerBehaviorTreeFromFile(main_xml);
            RCLCPP_INFO(get_logger(), "Main BT XML loaded from: %s", main_xml.c_str());
        }
        catch (const BT::RuntimeError &e)
        {
            RCLCPP_ERROR(get_logger(), "BT factory error: %s", e.what());
            if (setup_timer_)
            {
                setup_timer_->cancel();
            }
            return;
        }

        bt_factory_initialized_ = true;
        std::string xml_models = BT::writeTreeNodesModelXML(factory_);
        std::ofstream out("tree_nodes_model.xml");
        out << xml_models;

        if (setup_timer_)
        {
            setup_timer_->cancel();
        }
        RCLCPP_INFO(get_logger(), "BehaviorTreeFactory setup complete");

        cr_interfaces::msg::Log init_msg;
        init_msg.main_msg = "Waiting for a request...";
        init_msg.target_id = -1;
        gui_log_pub_->publish(init_msg);
    }

    rclcpp_action::GoalResponse OrchestratorNode::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ExecuteWorkflow::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received goal request for object %d", goal->object_id);

        if (!bt_factory_initialized_)
        {
            RCLCPP_ERROR(get_logger(), "BT Factory not initialized. Rejecting goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (bt_running_)
        {
            RCLCPP_WARN(get_logger(), "BT already running. Rejecting new goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        (void)uuid;
        cr_interfaces::msg::Log msg;
        msg.main_msg = "Request accepted!";
        msg.target_id = goal->object_id;
        gui_log_pub_->publish(msg);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse OrchestratorNode::handle_cancel(
        const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
    {
        RCLCPP_INFO(
            get_logger(),
            "Received request to cancel goal ID: %s",
            rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void OrchestratorNode::execute(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
    {
        RCLCPP_INFO(
            get_logger(),
            "Executing goal for object %d in new thread",
            goal_handle->get_goal()->object_id);

        bt_running_ = true; // ✅ segna in esecuzione

        std::thread bt_executor_thread([this, goal_handle]()
                                       {
                                           auto result = std::make_shared<ExecuteWorkflow::Result>();

                                           auto thread_local_blackboard = BT::Blackboard::create();
                                           thread_local_blackboard->set<rclcpp::Node::SharedPtr>("ros_node", shared_from_this());

                                           auto motion_commander = std::make_shared<cr::motion_core::MotionCommander>(
                                               shared_from_this(), "arm_manipulator", "gripper");
                                           thread_local_blackboard->set("motion_commander", motion_commander);
                                           thread_local_blackboard->set<std::string>("object_id", std::to_string(goal_handle->get_goal()->object_id));
                                           thread_local_blackboard->set("gui_log_pub", gui_log_pub_);

                                           loadConfigurationToBlackboard(thread_local_blackboard);

                                           BT::Tree local_tree;
                                           try
                                           {
                                               RCLCPP_INFO(get_logger(), "BT Thread: Creating tree 'PickAndPlaceWorkflow'");
                                               local_tree = factory_.createTree("PickAndPlaceWorkflow", thread_local_blackboard);
                                           }
                                           catch (const BT::RuntimeError &e)
                                           {
                                               RCLCPP_ERROR(get_logger(), "BT Thread: Error creating Behavior Tree: %s", e.what());
                                               result->success = false;
                                               goal_handle->abort(result);
                                               bt_running_ = false;
                                               publishWaitingMessage(); // ✅
                                               return;
                                           }
                                           catch (const std::exception &e)
                                           {
                                               RCLCPP_ERROR(get_logger(), "BT Thread: Exception creating Behavior Tree: %s", e.what());
                                               result->success = false;
                                               goal_handle->abort(result);
                                               bt_running_ = false;
                                               publishWaitingMessage(); // ✅
                                               return;
                                           }

                                           RCLCPP_INFO(get_logger(), "BT Thread: Starting execution for object %d", goal_handle->get_goal()->object_id);

                                           BT::NodeStatus status = BT::NodeStatus::RUNNING;
                                           rclcpp::Rate loop_rate(10);

                                           while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
                                           {
                                               if (goal_handle->is_canceling())
                                               {
                                                   RCLCPP_INFO(get_logger(), "BT Thread: Goal canceled, halting tree");
                                                   local_tree.haltTree();
                                                   break;
                                               }

                                               try
                                               {
                                                   status = local_tree.tickOnce();
                                               }
                                               catch (const std::exception &e)
                                               {
                                                   RCLCPP_ERROR(get_logger(), "BT Thread: Exception during tick: %s", e.what());
                                                   status = BT::NodeStatus::FAILURE;
                                                   break;
                                               }

                                               loop_rate.sleep();
                                           }

                                           if (goal_handle->is_canceling())
                                           {
                                               result->success = false;
                                               goal_handle->canceled(result);
                                               RCLCPP_INFO(get_logger(), "BT Thread: Workflow canceled");
                                           }
                                           else if (status == BT::NodeStatus::SUCCESS)
                                           {
                                               result->success = true;
                                               goal_handle->succeed(result);
                                               RCLCPP_INFO(get_logger(), "BT Thread: Workflow succeeded");
                                           }
                                           else
                                           {
                                               result->success = false;
                                               goal_handle->abort(result);
                                               RCLCPP_ERROR(get_logger(), "BT Thread: Workflow failed");
                                           }

                                           // ✅ Fine esecuzione: sblocca accettazione goal e ristampa "waiting"
                                           bt_running_ = false;
                                           publishWaitingMessage(); // ✅
                                       });

        bt_executor_thread.detach();
    }

    void OrchestratorNode::loadConfigurationToBlackboard(BT::Blackboard::Ptr blackboard)
    {
        blackboard->set("pre_approach_distance", pre_approach_distance_);
        blackboard->set("approach_distance", approach_distance_);
        blackboard->set("gripper_open", gripper_open_value_);
        blackboard->set("gripper_close", gripper_close_value_);
        blackboard->set("home_z_position", home_z_position_);
        blackboard->set("place_offset_x", place_offset_x_);
        blackboard->set("place_offset_z", place_offset_z_);

        RCLCPP_DEBUG(get_logger(), "Configuration loaded to blackboard");
    }

    void OrchestratorNode::publishWaitingMessage()
    {
        cr_interfaces::msg::Log msg;
        msg.main_msg = "Waiting for a request...";
        msg.target_id = -1;
        gui_log_pub_->publish(msg);
    }

} // namespace cr::bt::orchestrator
