/**
 * @file static_scene_publisher.hpp
 * @brief Defines a node for publishing static objects to the MoveIt planning scene.
 *
 * @ingroup cr_scene_management_nodes
 */

#ifndef CR_SCENE_MANAGEMENT_STATIC_SCENE_PUBLISHER_HPP_
#define CR_SCENE_MANAGEMENT_STATIC_SCENE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace cr::scene_management
{

    /**
     * @class StaticScenePublisher
     * @brief A node that publishes static collision objects to the MoveIt planning scene.
     *
     * It retrieves the PlanningSceneMonitor using the SceneManager singleton,
     * builds collision objects (in this project only the table), and publishes
     * them once as a PlanningScene diff.
     */
    class StaticScenePublisher : public rclcpp::Node
    {
    public:
        /**
         * @brief Construct a StaticScenePublisher node.
         *
         * This constructor declares the `/planning_scene` publisher and starts
         * a wall timer that will invoke `publishStaticScene()` after a short delay,
         * which ensures that all required components have been properly
         * initialized before publishing the scene.
         *
         * @param options NodeOptions for ROS2 configuration.
         */
        explicit StaticScenePublisher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        /**
         * @brief Default destructor.
         */
        ~StaticScenePublisher() override = default;

    private:
        /**
         * @brief Load collision objects and publish a PlanningScene diff.
         *
         * This method retrieves the PlanningSceneMonitor from the SceneManager
         * singleton, constructs collision objects for every static item (in our
         * case the table mesh in the "world" frame), updates the allowed collision
         * matrix to allow necessary interactions, and publishes the resulting
         * PlanningScene diff to MoveIt.
         */
        void publishStaticScene();

        /// Timer used to delay the static scene publication after node startup
        rclcpp::TimerBase::SharedPtr timer_;

        /// Publisher for PlanningScene messages to MoveIt
        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
    };

} // namespace cr::scene_management

#endif // CR_SCENE_MANAGEMENT_STATIC_SCENE_PUBLISHER_HPP_