#include "cr_bt_orchestrator/custom_bt_nodes.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>

class BtOrchestratorNode : public rclcpp::Node
{
public:
  BtOrchestratorNode() : Node("bt_orchestrator_node")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing BT Orchestrator Node");

    factory_.registerNodeType<cr::bt_nodes::GetObjectInfo>("GetObjectInfo");
    factory_.registerNodeType<cr::bt_nodes::ExecutePick>("ExecutePick");
    factory_.registerNodeType<cr::bt_nodes::ExecutePlace>("ExecutePlace");
    factory_.registerNodeType<cr::bt_nodes::LogMessage>("LogSuccess");

    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("cr_bt_orchestrator");
    std::string bt_xml_path =
        package_share_directory + "/bt_xml/simple_pick_place.xml";

    RCLCPP_INFO(this->get_logger(), "Loading BT from: %s", bt_xml_path.c_str());

    auto blackboard = BT::Blackboard::create();
    tree_ = factory_.createTreeFromFile(bt_xml_path, blackboard);

    // Aggiunta logger
    // Logger su console
    stdout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
    // Logger per Groot2 (visualizzazione)
    std::string groot_log_path = package_share_directory + "/bt_trace.btlog";
    FILE *f = fopen(groot_log_path.c_str(), "w");
    if (f)
      fclose(f);
    groot_logger_ = std::make_unique<BT::FileLogger2>(tree_, groot_log_path);

    RCLCPP_INFO(this->get_logger(),
                "BT Orchestrator Node initialized. Ready to execute.");

    blackboard->set("target_object_id", std::string("cubetto_rosso"));

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&BtOrchestratorNode::execute_bt_once, this));
  }

private:
  void execute_bt_once()
  {
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Executing Behavior Tree...");
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    status = tree_.tickWhileRunning();

    RCLCPP_INFO(this->get_logger(),
                "Behavior Tree execution finished with status: %s",
                BT::toStr(status).c_str());
  }

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<BT::StdCoutLogger> stdout_logger_;
  std::unique_ptr<BT::FileLogger2> groot_logger_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BtOrchestratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
