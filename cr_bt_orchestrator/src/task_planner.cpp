#include "cr_bt_orchestrator/task_planner.hpp"
#include <cr_bt_orchestrator/custom_bt_nodes.hpp>

#include <cr_bt_pick_place/set_gripper_node.hpp>
#include <cr_bt_pick_place/arm_horizontal_move_node.hpp>
#include <cr_bt_pick_place/arm_vertical_move_node.hpp>
#include <cr_bt_pick_place/set_collision_allowed_node.hpp>
#include <cr_bt_pick_place/set_object_attached_node.hpp>

#include <cr_motion_core/motion_commander.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace cr
{
    namespace bt_orchestrator
    {
        BtOrchestratorNode::BtOrchestratorNode(const rclcpp::NodeOptions &options)
            : Node("bt_orchestrator_node", options)
        {
            RCLCPP_INFO(get_logger(), "Initializing BT Orchestrator Node");

            execute_workflow_server_ = rclcpp_action::create_server<ExecuteWorkflow>(
                this,
                "cr/execute_workflow",
                std::bind(&BtOrchestratorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&BtOrchestratorNode::handle_cancel, this, std::placeholders::_1),
                std::bind(&BtOrchestratorNode::execute, this, std::placeholders::_1));

            setup_timer_ = create_wall_timer(
                0ms,
                std::bind(&BtOrchestratorNode::setupBTFactory, this));
        }

        void BtOrchestratorNode::setupBTFactory()
        {
            if (bt_factory_initialized_)
            {
                return;
            }

            RCLCPP_INFO(get_logger(), "Setting up BehaviorTreeFactory...");

            // A) Registra i nodi custom (come prima)
            // ... (tutta la tua registrazione dei nodi custom) ...
            BT::RosNodeParams default_ros_params;
            default_ros_params.nh = shared_from_this();

            BT::RosNodeParams freeze_params = default_ros_params;
            freeze_params.default_port_value = "cr/freeze_scene";
            factory_.registerNodeType<cr::bt_nodes::FreezeScene>("FreezeScene", freeze_params);

            BT::RosNodeParams service_params = default_ros_params;
            service_params.default_port_value = "cr/get_object_info";
            factory_.registerNodeType<cr::bt_nodes::GetObjectInfo>("GetObjectInfo", service_params);

            BT::RosNodeParams collision_params = default_ros_params;
            collision_params.default_port_value = "/allow_collision";
            factory_.registerNodeType<cr::bt::pick_place::SetCollisionAllowedNode>("SetCollisionAllowed", collision_params);

            BT::RosNodeParams attach_params = default_ros_params;
            attach_params.default_port_value = "/attach_object";
            factory_.registerNodeType<cr::bt::pick_place::SetObjectAttachedNode>("SetObjectAttached", attach_params);

            factory_.registerNodeType<cr::bt_nodes::IsAreaSafe>("IsAreaSafe");
            factory_.registerNodeType<cr::bt_nodes::EnsureAreaIsSafe>("EnsureAreaIsSafe");
            factory_.registerNodeType<cr::bt::pick_place::SetGripperNode>("SetGripper");
            factory_.registerNodeType<cr::bt::pick_place::ArmVerticalMoveNode>("ArmVerticalMove");
            factory_.registerNodeType<cr::bt::pick_place::ArmHorizontalMoveNode>("ArmHorizontalMove");
            factory_.registerNodeType<cr::bt_nodes::LogMessage>("LogMessage");

            // B) Registra TUTTI i file XML necessari (Albero Principale e SubTrees)
            RCLCPP_INFO(get_logger(), "Registering BehaviorTree XML files...");
            try
            {
                // Registra il file XML dell'ALBERO PRINCIPALE
                const auto orchestrator_pkg_share = ament_index_cpp::get_package_share_directory("cr_bt_orchestrator");
                const auto main_tree_xml_path = orchestrator_pkg_share + "/bt_xml/simple_pick_place.xml";
                factory_.registerBehaviorTreeFromFile(main_tree_xml_path);
                RCLCPP_INFO(get_logger(), "Registered Main Tree XML from: %s", main_tree_xml_path.c_str());

                // Registra il SubTree di Pick dal pacchetto cr_bt_pick_place
                const auto pick_place_pkg_share = ament_index_cpp::get_package_share_directory("cr_bt_pick_place");
                const auto pick_subtree_xml_path = pick_place_pkg_share + "/bt_xml/pick_subtree.xml";
                factory_.registerBehaviorTreeFromFile(pick_subtree_xml_path);
                RCLCPP_INFO(get_logger(), "Registered Pick SubTree from: %s", pick_subtree_xml_path.c_str());

                // Registra il SubTree di Place dal pacchetto cr_bt_pick_place
                const auto place_subtree_xml_path = pick_place_pkg_share + "/bt_xml/place_subtree.xml";
                factory_.registerBehaviorTreeFromFile(place_subtree_xml_path);
                RCLCPP_INFO(get_logger(), "Registered Place SubTree from: %s", place_subtree_xml_path.c_str());
            }
            catch (const BT::RuntimeError &e)
            {
                RCLCPP_ERROR(get_logger(), "Error registering BehaviorTree XMLs: %s", e.what());
                if (setup_timer_)
                    setup_timer_->cancel();
                return;
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), "Generic exception registering BehaviorTree XMLs: %s", e.what());
                if (setup_timer_)
                    setup_timer_->cancel();
                return;
            }

            bt_factory_initialized_ = true;
            if (setup_timer_)
                setup_timer_->cancel();

            RCLCPP_INFO(get_logger(), "BehaviorTreeFactory setup complete, all XMLs registered.");
        }

        rclcpp_action::GoalResponse BtOrchestratorNode::handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ExecuteWorkflow::Goal> goal)
        {
            RCLCPP_INFO(get_logger(), "Received goal request for object %d", goal->object_id);
            if (!bt_factory_initialized_)
            {
                RCLCPP_ERROR(get_logger(), "BT Factory not initialized yet or SubTree registration failed. Rejecting goal.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse BtOrchestratorNode::handle_cancel(
            const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Received request to cancel goal ID: %s",
                        rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void BtOrchestratorNode::execute(
            const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Executing goal for object %d in a new thread", goal_handle->get_goal()->object_id);

            auto factory_ptr = &factory_; // Puntatore alla factory membro

            std::thread bt_executor_thread([this, goal_handle, factory_ptr]()
                                           {
                auto result = std::make_shared<ExecuteWorkflow::Result>();
                // ... (feedback) ...

                auto thread_local_blackboard = BT::Blackboard::create();
                // ... (popolamento della blackboard locale come prima) ...
                thread_local_blackboard->set<rclcpp::Node::SharedPtr>("ros_node", shared_from_this());
                auto motion_commander = std::make_shared<cr::motion_core::MotionCommander>(
                    shared_from_this(), "arm_manipulator", "gripper");
                thread_local_blackboard->set("motion_commander", motion_commander);
                thread_local_blackboard->set<std::string>("object_id", std::to_string(goal_handle->get_goal()->object_id));
                thread_local_blackboard->set("pre_approach_distance", 0.15);
                thread_local_blackboard->set("approach_distance", 0.06);
                thread_local_blackboard->set("gripper_close", 0.8);
                thread_local_blackboard->set("gripper_open", 0.0);
                thread_local_blackboard->set("place_offset_z", 0.1);
                thread_local_blackboard->set("place_offset_x", 0.1);



                BT::Tree local_tree;
                try {
                    // Ora crea l'albero principale usando il suo ID
                    // La factory cercherà l'ID "PickAndPlace" tra quelli registrati
                    // e quando incontrerà <SubTree ID="Pick"/>, cercherà anche "Pick".
                    RCLCPP_INFO(get_logger(), "BT Thread: Creating tree with ID 'PickAndPlace'");
                    local_tree = factory_ptr->createTree("GuardedPickAndPlace", thread_local_blackboard);

                } catch (const BT::RuntimeError& e) {
                    RCLCPP_ERROR(get_logger(), "BT Thread: Error creating Behavior Tree: %s", e.what());
                    result->success = false;
                    goal_handle->abort(result);
                    return;
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(get_logger(), "BT Thread: Generic exception creating Behavior Tree: %s", e.what());
                    result->success = false;
                    goal_handle->abort(result);
                    return;
                }

                // ... (resto del loop di tick e gestione del risultato come prima) ...
                RCLCPP_INFO(get_logger(), "BT Thread: Starting tick loop for object %d", goal_handle->get_goal()->object_id);
                BT::NodeStatus status = BT::NodeStatus::RUNNING;
                rclcpp::Rate loop_rate(10);

                while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
                    if (goal_handle->is_canceling()) {
                        RCLCPP_INFO(get_logger(), "BT Thread: Goal for object %d canceled, halting tree.", goal_handle->get_goal()->object_id);
                        local_tree.haltTree();
                        status = BT::NodeStatus::IDLE; 
                        break; 
                    }
                    try {
                        status = local_tree.tickOnce();
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(get_logger(), "BT Thread: Exception during tree.tickOnce(): %s", e.what());
                        status = BT::NodeStatus::FAILURE; 
                        break;
                    }
                    loop_rate.sleep();
                }

                if (goal_handle->is_canceling()) { 
                    result->success = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(get_logger(), "BT Thread: Workflow for object %d CANCELED.", goal_handle->get_goal()->object_id);
                } else if (status == BT::NodeStatus::SUCCESS) {
                    result->success = true;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(get_logger(), "BT Thread: Workflow for object %d SUCCEEDED.", goal_handle->get_goal()->object_id);
                } else {
                    result->success = false;
                    goal_handle->abort(result);
                    RCLCPP_ERROR(get_logger(), "BT Thread: Workflow for object %d FAILED with status: %s",
                                 goal_handle->get_goal()->object_id, BT::toStr(status).c_str());
                } });

            bt_executor_thread.detach();
        }
    } // namespace bt_orchestrator
} // namespace cr
