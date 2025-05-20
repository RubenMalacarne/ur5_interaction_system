#include "cr_bt_orchestrator/task_planner.hpp"
#include <cr_bt_orchestrator/custom_bt_nodes.hpp>
#include <cr_bt_pick_place/set_gripper_node.hpp>
#include <cr_motion_core/motion_commander.hpp>

using namespace std::chrono_literals;

namespace cr
{
    namespace bt_orchestrator
    {
        BtOrchestratorNode::BtOrchestratorNode(const rclcpp::NodeOptions &options)
            : Node("bt_orchestrator_node", options)
        {
            RCLCPP_INFO(get_logger(), "Initializing BT Orchestrator Node");

            // 1) Action server
            execute_workflow_server_ = rclcpp_action::create_server<ExecuteWorkflow>(
                this,
                "cr/execute_workflow",
                std::bind(&BtOrchestratorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&BtOrchestratorNode::handle_cancel, this, std::placeholders::_1),
                std::bind(&BtOrchestratorNode::execute, this, std::placeholders::_1));

            // 2) Timer a 0ms per post‐costructor setup
            setup_timer_ = create_wall_timer(
                0ms,
                std::bind(&BtOrchestratorNode::setupBT, this));
        }

        void BtOrchestratorNode::setupBT()
        {
            if (bt_initialized_)
            {
                return;
            }
            bt_initialized_ = true;
            setup_timer_->cancel();

            // A) registra i nodi custom, incluso il servizio
            // –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

            // Service: FreezeScene
            BT::RosNodeParams freeze_params;
            freeze_params.nh = shared_from_this();
            // nome del servizio
            freeze_params.default_port_value = "cr/freeze_scene";
            factory_.registerNodeType<cr::bt_nodes::FreezeScene>("FreezeScene", freeze_params);


            // Service: GetObjectInfo
            BT::RosNodeParams service_params;
            service_params.nh = shared_from_this();
            // nome del servizio
            service_params.default_port_value = "cr/get_object_info";
            factory_.registerNodeType<cr::bt_nodes::GetObjectInfo>("GetObjectInfo", service_params);

            // Action: ExecutePick
            BT::RosNodeParams pick_params;
            pick_params.nh = shared_from_this();
            // nome dell'action server per il pick
            pick_params.default_port_value = "cr/pick_action";
            factory_.registerNodeType<cr::bt_nodes::ExecutePick>("ExecutePick", pick_params);

            // Action: ExecutePlace
            BT::RosNodeParams place_params;
            place_params.nh = shared_from_this();
            // nome dell'action server per il place
            place_params.default_port_value = "cr/place_action";
            factory_.registerNodeType<cr::bt_nodes::ExecutePlace>("ExecutePlace", place_params);
 
            factory_.registerNodeType<cr::bt::pick_place::SetGripperNode>("SetGripper");

            // Nodo di logging (SyncActionNode non ha params)
            factory_.registerNodeType<cr::bt_nodes::LogMessage>("LogSuccess");

            // B) carica l’XML
            const auto pkg_share = ament_index_cpp::get_package_share_directory("cr_bt_orchestrator");
            const auto bt_xml = pkg_share + "/bt_xml/simple_pick_place.xml";
            RCLCPP_INFO(get_logger(), "Loading BT from: %s", bt_xml.c_str());

            blackboard_ = BT::Blackboard::create();
            blackboard_->set<rclcpp::Node::SharedPtr>("ros_node", shared_from_this());

            auto motion_commander = std::make_shared<cr::motion_core::MotionCommander>(
                shared_from_this(),  // Passa il nodo ROS
                "arm_manipulator",      // Nome del gruppo braccio (se diverso)
                "gripper"             // Nome del gruppo gripper (se diverso)
            );

            // Inserisci l'istanza nella blackboard
            blackboard_->set("motion_commander", motion_commander);
            tree_ = factory_.createTreeFromFile(bt_xml, blackboard_);

            // C) logger
            stdout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
            const auto log_path = pkg_share + "/bt_trace.btlog";
            FILE *f = fopen(log_path.c_str(), "w");
            if (f) { fclose(f); }
            groot_logger_ = std::make_unique<BT::FileLogger2>(tree_, log_path);

            RCLCPP_INFO(get_logger(), "BT Orchestrator Node initialized. Ready to execute.");
        }


        rclcpp_action::GoalResponse BtOrchestratorNode::handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ExecuteWorkflow::Goal> goal)
        {
            RCLCPP_INFO(get_logger(), "Received goal request for object %d", goal->object_id);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse BtOrchestratorNode::handle_cancel(
            const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void BtOrchestratorNode::execute(
            const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Executing goal");

            auto result = std::make_shared<ExecuteWorkflow::Result>();
            auto feedback = std::make_shared<ExecuteWorkflow::Feedback>();

            // passaggio su blackboard (l’XML usa {target_object_id})
            blackboard_->set<std::string>(
                "object_id",
                std::to_string(goal_handle->get_goal()->object_id));

            geometry_msgs::msg::Point target_position;
            target_position.x = 0.150; 
            target_position.y = 0.625;
            target_position.z = 0.939;
            blackboard_->set<geometry_msgs::msg::Point>(
                "target_position",
                target_position);

            // loop di tick
            BT::NodeStatus status = BT::NodeStatus::RUNNING;
            while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
            {
                if (goal_handle->is_canceling())
                {
                    RCLCPP_INFO(get_logger(), "Goal canceled");
                    goal_handle->canceled(result);
                    return;
                }
                status = tree_.tickOnce();
                goal_handle->publish_feedback(feedback);
                std::this_thread::sleep_for(10ms);
            }

            if (status == BT::NodeStatus::SUCCESS)
            {
                RCLCPP_INFO(get_logger(), "Workflow succeeded");
                goal_handle->succeed(result);
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Workflow failed");
                goal_handle->abort(result);
            }
        }
    } // namespace bt_orchestrator
} // namespace cr
