#ifndef CR_SCENE_MANAGEMENT_SCENE_MANAGER_HPP
#define CR_SCENE_MANAGEMENT_SCENE_MANAGER_HPP

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/rclcpp.hpp>

namespace cr {
namespace scene_management {

    class SceneManager
    {
    public:

        static SceneManager& instance(const rclcpp::Node::SharedPtr& node = nullptr);

        SceneManager(const SceneManager&) = delete;
        SceneManager& operator=(const SceneManager&) = delete;

        planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor() const
        {
            return planning_scene_monitor_;
        }

    private:
        
        SceneManager(const rclcpp::Node::SharedPtr& node);

        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    };

}  // namespace scene_management
}  // namespace cr

#endif // CR_SCENE_MANAGEMENT_SCENE_MANAGER_HPP
