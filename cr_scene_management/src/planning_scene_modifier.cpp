/**
 * @file planning_scene_modifier.cpp
 * @brief Implementation of the PlanningSceneModifier class, responsible for modifying the MoveIt planning scene.
 */

#include "cr_scene_management/planning_scene_modifier.hpp"
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <cr_scene_management/scene_manager.hpp>

namespace cr {
namespace scene_management {

    /**
     * @brief Constructor for the PlanningSceneModifier class.
     * 
     */
    PlanningSceneModifier::PlanningSceneModifier(const rclcpp::NodeOptions& options)
        : Node("planning_scene_modifier", options)
    {
        
        // Publisher for sending planning scene diffs
        planning_scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);
        
        // Configure QoS profile
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_profile.reliable();
        qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        qos_profile.keep_last(10);

        // Subscriber su /object_info
        obj_detection_result_sub_ = this->create_subscription<cr_interfaces::msg::ObjectInfoArray>(
            "cr/scene_objects", qos_profile,
            std::bind(&PlanningSceneModifier::spawnObjects, this, std::placeholders::_1));

        // Subscribe to object detection topic
        allow_collision_srv_ = this->create_service<cr_interfaces::srv::AllowCollision>(
            "/allow_collision",
            std::bind(&PlanningSceneModifier::allowCollision, this,
                        std::placeholders::_1, std::placeholders::_2));

        // Service to attach an object
        attach_object_srv_ = this->create_service<cr_interfaces::srv::AttachObject>(
            "/attach_object",
            std::bind(&PlanningSceneModifier::attachObject, this,
                        std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "PlanningSceneModifier is ready.");
    }

    /**
     * @brief Callback for spawning new collision objects into the planning scene.
     * 
     * @param detected_objects Message containing an array of detected objects to be added.
     */
    void PlanningSceneModifier::spawnObjects(const cr_interfaces::msg::ObjectInfoArray::SharedPtr detected_objects)
    {
        RCLCPP_INFO(this->get_logger(), "Received %lu objects to spawn.", detected_objects->objects.size());

        moveit_msgs::msg::PlanningScene planning_scene;

        for (const auto& obj : detected_objects->objects) {

            RCLCPP_INFO(this->get_logger(), "Adding object %d to planning scene msg...", obj.id);

            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.id = std::to_string(obj.id);
            collision_object.header.frame_id = "world";

            geometry_msgs::msg::Pose pose;
            pose.position.x = obj.center.x;
            pose.position.y = obj.center.y;
            pose.position.z = obj.center.z;

            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = obj.size.x;
            primitive.dimensions[1] = obj.size.y;
            primitive.dimensions[2] = obj.size.z;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose);
            collision_object.operation = collision_object.ADD;

            planning_scene.world.collision_objects.push_back(collision_object);
        }

        planning_scene.is_diff = true;
        planning_scene_pub_->publish(planning_scene);
        RCLCPP_INFO(this->get_logger(), "Objects spawned in the scene.");
    }

    /**
     * @brief Service callback to modify the Allowed Collision Matrix (ACM).
     * 
     * @param request Service request containing object ID and permission flag.
     * @param response Service response indicating success or failure.
     */
    void PlanningSceneModifier::allowCollision(
        const std::shared_ptr<cr_interfaces::srv::AllowCollision::Request> request,
        std::shared_ptr<cr_interfaces::srv::AllowCollision::Response> response)
    {
        auto& manager = cr::scene_management::SceneManager::instance(shared_from_this());
        auto psm = manager.getPlanningSceneMonitor();
    
        if (!psm) {
            RCLCPP_ERROR(get_logger(), "[allowCollision] PlanningSceneMonitor è nullo.");
            response->success = false;
            return;
        }
    
        if (!psm->getPlanningScene()) {
            RCLCPP_ERROR(get_logger(), "[allowCollision] PlanningScene non disponibile!");
            response->success = false;
            return;
        }
    
        RCLCPP_DEBUG(get_logger(), "[allowCollision] PlanningSceneMonitor e PlanningScene trovati. Procedo con la modifica dell'ACM.");
    
        // Lock di scrittura
        planning_scene_monitor::LockedPlanningSceneRW locked_scene(psm);
        collision_detection::AllowedCollisionMatrix& acm = locked_scene->getAllowedCollisionMatrixNonConst();
    
        std::vector<std::string> link_names = {
            "robotiq_85_base_link",
            "robotiq_85_left_knuckle_link",
            "robotiq_85_right_knuckle_link",
            "robotiq_85_left_finger_link",
            "robotiq_85_right_finger_link",
            "robotiq_85_left_inner_knuckle_link",
            "robotiq_85_right_inner_knuckle_link",
            "robotiq_85_left_finger_tip_link",
            "robotiq_85_right_finger_tip_link"
        };
    
        for (const auto& link : link_names) {
            std::string object_id_str = std::to_string(request->object_id);
            acm.setEntry(link, object_id_str, request->is_allowed);
            RCLCPP_DEBUG(get_logger(), "[allowCollision] Set entry: [%s] <-> [%d] = %s",
                         link.c_str(), request->object_id,
                         request->is_allowed ? "ALLOWED" : "NOT ALLOWED");
        }
    
        moveit_msgs::msg::PlanningScene scene_msg;
        scene_msg.is_diff = true;
    
        acm.getMessage(scene_msg.allowed_collision_matrix);
    
        // Check if publisher is ready
        if (!planning_scene_pub_) {
            RCLCPP_ERROR(get_logger(), "[allowCollision] planning_scene_pub_ non inizializzato!");
            response->success = false;
            return;
        }
    
        planning_scene_pub_->publish(scene_msg);
        RCLCPP_INFO(get_logger(), "[allowCollision] Pubblicata nuova ACM per '%d'. Collisioni %s con i link del gripper.",
                    request->object_id,
                    request->is_allowed ? "PERMESSE" : "VIETATE");
        response->success = true;
    }
    
    /**
     * @brief Service callback to attach or detach a collision object to/from the robot.
     * 
     * @param request Service request containing object ID and attach flag.
     * @param response Service response indicating success.
     */
    void PlanningSceneModifier::attachObject(
        const std::shared_ptr<cr_interfaces::srv::AttachObject::Request> request,
        std::shared_ptr<cr_interfaces::srv::AttachObject::Response> response)
    {
        if(request->attach)
        {
            // Remove object from world
            moveit_msgs::msg::CollisionObject remove_object;
            remove_object.id = std::to_string(request->object_id);
            remove_object.header.frame_id = "world";
            remove_object.operation = remove_object.REMOVE;

            // Attach object to robot
            moveit_msgs::msg::AttachedCollisionObject attached_object;
            attached_object.link_name = "tool0";
            attached_object.object.header.frame_id = "tool0";
            attached_object.object.id = std::to_string(request->object_id);

            attached_object.touch_links = std::vector<std::string>{
                "robotiq_85_base_link",
                "robotiq_85_left_knuckle_link",
                "robotiq_85_right_knuckle_link",
                "robotiq_85_left_finger_link",
                "robotiq_85_right_finger_link",
                "robotiq_85_left_inner_knuckle_link",
                "robotiq_85_right_inner_knuckle_link",
                "robotiq_85_left_finger_tip_link",
                "robotiq_85_right_finger_tip_link"
            };  

            moveit_msgs::msg::PlanningScene planning_scene;
        
            planning_scene.world.collision_objects.clear();
            planning_scene.world.collision_objects.push_back(remove_object);
            planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
            planning_scene.is_diff = true;
            planning_scene.robot_state.is_diff = true;
            planning_scene_pub_->publish(planning_scene);

            RCLCPP_INFO(this->get_logger(), "Object successfully attached to robot.");

            response->success = true;
        } 
        else 
        {
            // Stacchiamo l'oggetto dal robot
            moveit_msgs::msg::AttachedCollisionObject detach_object;
            detach_object.object.id = std::to_string(request->object_id);
            detach_object.link_name = "tool0";
            detach_object.object.operation = detach_object.object.REMOVE;

            moveit_msgs::msg::CollisionObject world_object = detach_object.object;
            world_object.operation = world_object.ADD;

            RCLCPP_INFO(this->get_logger(), "Detaching the object from the robot and returning it to the world.");

            moveit_msgs::msg::PlanningScene planning_scene;
            planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
            planning_scene.robot_state.is_diff = true;
            planning_scene.world.collision_objects.push_back(world_object);
            planning_scene.is_diff = true;
            planning_scene_pub_->publish(planning_scene);

            RCLCPP_INFO(this->get_logger(), "Object successfully detached from robot.");

            response->success = true;
        }
    }

} // namespace scene_management
} // namespace cr

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cr::scene_management::PlanningSceneModifier)
