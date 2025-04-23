#include "cr_scene_management/scene_manager.hpp"

namespace cr {
namespace scene_management {

    SceneManager& SceneManager::instance(const rclcpp::Node::SharedPtr& node)
    {
        static SceneManager* s = nullptr;

        if (!s) {
            if (!node) {
                throw std::runtime_error(
                    "SceneManager::instance() chiamato la prima volta senza un nodo valido!"
                );
            }
            s = new SceneManager(node);
        }

        return *s;
    }

    SceneManager::SceneManager(const rclcpp::Node::SharedPtr& node)
    {

        planning_scene_monitor_ =
            std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                node, "robot_description");

        if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene()) {
            RCLCPP_ERROR(node->get_logger(),
                        "Impossibile inizializzare il PlanningSceneMonitor!");
            throw std::runtime_error("PlanningSceneMonitor non disponibile");
        }

        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->startStateMonitor();

        planning_scene_monitor_->startPublishingPlanningScene(
            planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
            "planning_scene"
        );

        RCLCPP_INFO(node->get_logger(),
                    "SceneManager creato e PlanningSceneMonitor avviato correttamente!");
    }

}  // namespace scene_management
}  // namespace cr
