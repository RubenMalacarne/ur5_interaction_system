#include "cr_scene_management/scene_manager.hpp"

namespace cr::scene_management {

    // Returns the singleton instance. Initializes it on first call with a valid node.
    SceneManager& SceneManager::instance(const rclcpp::Node::SharedPtr& node)
    {
        static SceneManager* s = nullptr;

        if (!s) {
            if (!node) {
                throw std::runtime_error(
                    "SceneManager::instance() called first time without a valid node!"
                );
            }
            s = new SceneManager(node);
        }

        return *s;
    }

    // Initializes the PlanningSceneMonitor with the given node
    SceneManager::SceneManager(const rclcpp::Node::SharedPtr& node)
    {
        planning_scene_monitor_ =
            std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                node, "robot_description");

        if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) {
            RCLCPP_ERROR(node->get_logger(),
                        "Failed to initialize PlanningSceneMonitor!");
            throw std::runtime_error("PlanningSceneMonitor not available");
        }

        // Start all required monitoring components
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();

        // Enable planning scene publishing
        planning_scene_monitor_->startPublishingPlanningScene(
            planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
            "planning_scene"
        );

        RCLCPP_INFO(node->get_logger(),
                    "SceneManager created and PlanningSceneMonitor successfully started.");
    }

}  // namespace cr::scene_management
