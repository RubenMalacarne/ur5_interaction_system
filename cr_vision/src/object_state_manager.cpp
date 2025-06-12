#include "cr_vision/object_state_manager.hpp"

namespace cr::vision
{

    ObjectStateManager::ObjectStateManager(const rclcpp::NodeOptions &options)
        : Node("object_state_manager", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting ObjectStateManager...");

        // Service to get info about a specific object
        get_object_info_srv_ = this->create_service<cr_interfaces::srv::GetObjectInfo>(
            "cr/get_object_info",
            std::bind(&ObjectStateManager::give_object_info, this, std::placeholders::_1, std::placeholders::_2));

        // Service to freeze/unfreeze the scene
        freeze_scene_srv_ = this->create_service<cr_interfaces::srv::FreezeScene>(
            "cr/freeze_scene",
            std::bind(&ObjectStateManager::handle_freeze_scene_request, this, std::placeholders::_1, std::placeholders::_2));

        // Subscription to incoming detected objects
        object_info_array_sub_ = this->create_subscription<cr_interfaces::msg::ObjectInfoArray>(
            "cr_vision/detected_objects",
            10,
            std::bind(&ObjectStateManager::update_scene_objects, this, std::placeholders::_1));

        // Publisher of internal scene object state
        scene_objects_pub_ = this->create_publisher<cr_interfaces::msg::ObjectInfoArray>("cr/scene_objects", 10);

        RCLCPP_INFO(this->get_logger(), "ObjectStateManager is ready.");
    }

    void ObjectStateManager::update_scene_objects(const cr_interfaces::msg::ObjectInfoArray &object_array)
    {
        if (is_scene_frozen_)
        {
            RCLCPP_WARN(this->get_logger(), "Scene is frozen. Skipping update.");
            return;
        }

        for (const auto &obj : object_array.objects)
        {
            // Add or update object in the internal map
            if (scene_objects_.find(obj.id) == scene_objects_.end())
            {
                RCLCPP_INFO(this->get_logger(), "New object added: id = %d", obj.id);
            }
            scene_objects_[obj.id] = obj;
        }

        scene_objects_pub_->publish(object_array);
    }

    void ObjectStateManager::give_object_info(
        const std::shared_ptr<cr_interfaces::srv::GetObjectInfo::Request> request,
        std::shared_ptr<cr_interfaces::srv::GetObjectInfo::Response> response)
    {
        const std::string &target_label = request->label;
        std::optional<int> min_id;

        for (const auto &[id, obj] : scene_objects_)
        {
            if (obj.label == target_label)
            {
                if (!min_id || id < *min_id)
                {
                    min_id = id;
                }
            }
        }

        if (min_id)
        {
            response->success = true;
            response->object_info = scene_objects_[*min_id];
            RCLCPP_INFO(this->get_logger(),
                        "Found object with label '%s' (id = %d).",
                        target_label.c_str(), *min_id);
        }
        else
        {
            response->success = false;
            RCLCPP_WARN(this->get_logger(),
                        "No object found with label '%s'.",
                        target_label.c_str());
        }
    }

    void ObjectStateManager::handle_freeze_scene_request(
        const std::shared_ptr<cr_interfaces::srv::FreezeScene::Request> request,
        std::shared_ptr<cr_interfaces::srv::FreezeScene::Response> response)
    {
        is_scene_frozen_ = request->freeze;

        if (is_scene_frozen_)
        {
            RCLCPP_INFO(this->get_logger(), "Scene is now frozen.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Scene is now unfrozen.");
        }

        response->success = true;
    }

} // namespace cr::vision
