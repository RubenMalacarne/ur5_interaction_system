// Node Responsibilities:
// 1. Depending on whether the scene should be frozen or not, it manages sending updates to the MoveIt planning_scene (Note: This node itself doesn't directly interact with MoveIt. It publishes the scene objects which another node might use for MoveIt updates).
// 2. Internally maintains a status table of the identified objects
// 3. Provides a service to request information about a specific object

#include "cr_vision/object_state_manager.hpp"

namespace cr {
namespace vision {
    /**
     * @brief Constructor for the ObjectStateManager node.
     *
     * Initializes the node, sets up subscriptions to object updates and freeze messages,
     * advertises a service to query object information, and sets up a publisher to broadcast
     * the current state of all known objects.
     *
     * Subscribes to:
     * - `cr_vision/object_selection_results` (cr_interfaces::msg::ObjectInfoArray): Incoming updates about detected objects.
     * - `cr/freeze_scene` (cr_interfaces::msg::FreezeScene): Command to freeze or unfreeze scene updates.
     *
     * Publishes to:
     * - `cr/scene_objects` (cr_interfaces::msg::ObjectInfoArray): The current set of all known objects in the scene.
     *
     * Provides a service:
     * - `cr/get_object_info` (cr_interfaces::srv::GetObjectInfo): Allows querying information about a specific object by its ID.
     *
     * @param options ROS 2 NodeOptions, used for composition or remapping.
     */
    ObjectStateManager::ObjectStateManager(const rclcpp::NodeOptions &options)
        : Node("object_state_manager", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting ObjectStateManager...");

        // Service to get information about a specific object
        this->get_object_info_srv_ = this->create_service<cr_interfaces::srv::GetObjectInfo>(
            "cr/get_object_info",
            std::bind(&ObjectStateManager::give_object_info, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "Service 'cr/get_object_info' has been created.");

        // Subscription to the array of detected objects
        this->object_info_array_sub_ = this->create_subscription<cr_interfaces::msg::ObjectInfoArray>(
            "cr_vision/object_selection_results",
            10,
            std::bind(&ObjectStateManager::update_scene_objects, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic 'cr_vision/object_selection_results'.");

        // Subscription to the freeze scene command
        this->freeze_scene_sub_ = this->create_subscription<cr_interfaces::msg::FreezeScene>(
            "cr/freeze_scene",
            10,
            std::bind(&ObjectStateManager::update_frozen_scene, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic 'cr/freeze_scene'.");

        // Publisher for the current state of all scene objects
        this->scene_objects_pub_ = this->create_publisher<cr_interfaces::msg::ObjectInfoArray>("cr/scene_objects", 10);
        RCLCPP_INFO(this->get_logger(), "Publisher 'cr/scene_objects' has been created.");

        RCLCPP_INFO(this->get_logger(), "ObjectStateManager is ready.");
    }


    /**
     * @brief Updates the internal object state and republishes the entire scene if the scene is not frozen.
     *
     * Iterates through the received `object_array`. For each object, it updates the corresponding entry
     * in the internal `scene_objects_` map. If a new object ID is encountered, it is added to the map.
     * If the scene is not currently frozen (`is_scene_frozen_` is false), it publishes the complete
     * `scene_objects_` map as a `cr_interfaces::msg::ObjectInfoArray` on the `cr/scene_objects` topic.
     *
     * @param object_array Message containing a list of updated object states.
     */
    void ObjectStateManager::update_scene_objects(const cr_interfaces::msg::ObjectInfoArray & object_array)
    {
        if (is_scene_frozen_)
        {
            RCLCPP_WARN(this->get_logger(), "Scene is frozen. Skipping object update and republish.");
            return;
        }

        for (const auto &obj : object_array.objects)
        {
            if (scene_objects_.find(obj.id) == scene_objects_.end())
            {
                scene_objects_[obj.id] = obj;
                RCLCPP_INFO(this->get_logger(), "New object added to the scene: id = %d", obj.id);
            }
            else
            {
                scene_objects_[obj.id] = obj;
                RCLCPP_DEBUG(this->get_logger(), "Updated information for object with id = %d.", obj.id);
            }
        }

        // Publish the current state of all scene objects
        cr_interfaces::msg::ObjectInfoArray current_scene_array;
        for (const auto& pair : scene_objects_) {
            current_scene_array.objects.push_back(pair.second);
        }
        this->scene_objects_pub_->publish(current_scene_array);
        RCLCPP_DEBUG(this->get_logger(), "Published updated scene objects. Total objects: %zu", current_scene_array.objects.size());
    }

    /**
     * @brief Service callback to provide details of a specific object by its ID.
     *
     * When a request is received on the `cr/get_object_info` service, this callback searches the
     * internal `scene_objects_` map for an object with the requested ID. If found, it populates
     * the response with the object's information and sets the `success` flag to true. If the object
     * ID is not found, it sets the `success` flag to false and logs a warning.
     *
     * @param request Pointer to the service request, containing the `id` of the object to query.
     * @param response Pointer to the service response, which will contain the `object_info` if found, and a `success` boolean.
     */
    void ObjectStateManager::give_object_info(const std::shared_ptr<cr_interfaces::srv::GetObjectInfo::Request> request,
                                 std::shared_ptr<cr_interfaces::srv::GetObjectInfo::Response> response)
    {
        auto it = scene_objects_.find(request->id);

        if (it != scene_objects_.end())
        {
            response->success = true;
            response->object_info = it->second;
            RCLCPP_INFO(this->get_logger(), "Object with id %d found and returned.", request->id);
        }
        else
        {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "Object with id %d not found in the scene.", request->id);
        }
    }

    /**
     * @brief Callback to update the frozen state of the scene.
     *
     * This callback is triggered when a message is received on the `cr/freeze_scene` topic.
     * It updates the internal `is_scene_frozen_` flag based on the `freeze` field of the incoming message.
     * When the scene is frozen, updates to the scene objects are still processed internally, but the
     * publishing of the updated scene to the `cr/scene_objects` topic is skipped.
     *
     * @param frozen_msg Message containing the new freeze state (true to freeze, false to unfreeze).
     */
    void ObjectStateManager::update_frozen_scene(const cr_interfaces::msg::FreezeScene & frozen_msg)
    {
        this->is_scene_frozen_ = frozen_msg.freeze;
        RCLCPP_INFO(this->get_logger(), "Scene freeze state updated to: %s", is_scene_frozen_ ? "frozen" : "unfrozen");
    }

} // namespace vision
} // namespace cr