/**
 * @file planning_scene_modifier.hpp
 * @brief Declares the PlanningSceneModifier node for managing dynamic updates to the MoveIt planning scene.
 *
 * This node handles runtime modifications to the scene such as adding or removing objects,
 * updating collision permissions, and attaching/detaching objects to the robot.
 *
 * @ingroup cr_scene_management_nodes
 */

#ifndef CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP_
#define CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cr_interfaces/msg/object_info_array.hpp>
#include <cr_interfaces/srv/allow_collision.hpp>
#include <cr_interfaces/srv/attach_object.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

namespace cr::scene_management
{

    /**
     * @class PlanningSceneModifier
     * @brief Node responsible for handling all dynamic updates to the MoveIt planning scene.
     *
     * It listens to object detection messages, manages collision permissions, and supports
     * attaching and detaching objects during manipulation.
     */
    class PlanningSceneModifier : public rclcpp::Node
    {
    public:
        /**
         * @brief Construct a PlanningSceneModifier node.
         *
         * Initializes publishers, subscribers, and services related to dynamic scene management.
         *
         * @param options ROS2 NodeOptions.
         */
        explicit PlanningSceneModifier(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        /**
         * @brief Default destructor.
         */
        ~PlanningSceneModifier() override = default;

    private:
        /**
         * @brief Callback for adding or removing detected objects in the planning scene.
         *
         * Updates the planning scene with newly detected objects and removes those no longer present.
         *
         * @param detected_objects Incoming array of detected object information.
         */
        void spawnObjects(const cr_interfaces::msg::ObjectInfoArray::SharedPtr detected_objects);

        /**
         * @brief Service callback to allow or deny collisions between the gripper and an object.
         *
         * Modifies the allowed collision matrix accordingly.
         *
         * @param request Object ID and collision flag.
         * @param response Operation result.
         */
        void allowCollision(
            const std::shared_ptr<cr_interfaces::srv::AllowCollision::Request> request,
            std::shared_ptr<cr_interfaces::srv::AllowCollision::Response> response);

        /**
         * @brief Service callback to attach or detach an object to/from the robot end-effector.
         *
         * Used during grasping or releasing phases.
         *
         * @param request Contains object ID and attach/detach flag.
         * @param response Operation result.
         */
        void attachObject(
            const std::shared_ptr<cr_interfaces::srv::AttachObject::Request> request,
            std::shared_ptr<cr_interfaces::srv::AttachObject::Response> response);

        /// Publisher for planning scene diffs
        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;

        /// Subscriber to object detection results
        rclcpp::Subscription<cr_interfaces::msg::ObjectInfoArray>::SharedPtr obj_detection_result_sub_;

        /// Service to allow or deny collisions with an object
        rclcpp::Service<cr_interfaces::srv::AllowCollision>::SharedPtr allow_collision_srv_;

        /// Service to attach or detach an object to/from the robot
        rclcpp::Service<cr_interfaces::srv::AttachObject>::SharedPtr attach_object_srv_;
    };

} // namespace cr::scene_management

#endif // CR_SCENE_MANAGEMENT_PLANNING_SCENE_MODIFIER_HPP_
