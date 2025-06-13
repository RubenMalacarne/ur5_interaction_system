/**
 * @file scene_manager.hpp
 * @brief Singleton wrapper for initializing and accessing the MoveIt PlanningSceneMonitor.
 *
 * This utility ensures a single instance of the PlanningSceneMonitor is shared
 * across nodes.
 *
 * @ingroup cr_scene_management_utils
 */

#ifndef CR_SCENE_MANAGEMENT_SCENE_MANAGER_HPP_
#define CR_SCENE_MANAGEMENT_SCENE_MANAGER_HPP_

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/rclcpp.hpp>

namespace cr::scene_management
{

    /**
     * @class SceneManager
     * @brief Provides a singleton interface to the PlanningSceneMonitor.
     *
     * The SceneManager ensures only one instance of PlanningSceneMonitor is initialized,
     * which can be accessed from any node in the system. On first use, a valid ROS node
     * must be passed to initialize the internal monitor.
     */
    class SceneManager
    {
    public:
        /**
         * @brief Get the singleton instance of the SceneManager.
         *
         * On first call, a valid node must be passed to initialize the internal
         * PlanningSceneMonitor. Subsequent calls will return the same instance.
         *
         * @param node The ROS 2 node used for initialization (only required once).
         * @return Reference to the SceneManager instance.
         */
        static SceneManager &instance(const rclcpp::Node::SharedPtr &node = nullptr);

        /// Deleted copy constructor.
        SceneManager(const SceneManager &) = delete;

        /// Deleted copy assignment operator.
        SceneManager &operator=(const SceneManager &) = delete;

        /**
         * @brief Get the internal PlanningSceneMonitor.
         * @return Shared pointer to the PlanningSceneMonitor instance.
         */
        planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor() const
        {
            return planning_scene_monitor_;
        }

    private:
        /// Private constructor used internally during singleton initialization.
        SceneManager(const rclcpp::Node::SharedPtr &node);

        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    };

} // namespace cr::scene_management

#endif // CR_SCENE_MANAGEMENT_SCENE_MANAGER_HPP_
