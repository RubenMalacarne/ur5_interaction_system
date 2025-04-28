// Node Responsibilities:
// 1. Depending on whether the scene should be frozen or not, it manages sending updates to the MoveIt planning_scene
// 2. Internally maintains a status table of the identified objects
// 3. Provides a service to request information about a specific object

#include "cr_vision/object_state_manager.hpp"

namespace cr {
namespace vision {

    ObjectStateManager::ObjectStateManager(const rclcpp::NodeOptions &options)
        : Node("object_state_manager", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting ObjectStateManager...");

        // Service
        this->get_object_info_srv_ = this->create_service<cr_interfaces::srv::GetObjectInfo>(
            "cr/get_object_info",
            std::bind(&ObjectStateManager::give_object_info, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Subscriptions
        this->object_info_array_sub_ = this->create_subscription<cr_interfaces::msg::ObjectInfoArray>(
            "cr_vision/object_selection_results",
            10,
            std::bind(&ObjectStateManager::update_scene_objects, this, std::placeholders::_1)
        );

        this->freeze_scene_sub_ = this->create_subscription<cr_interfaces::msg::FreezeScene>(
            "cr/freeze_scene",
            10,
            std::bind(&ObjectStateManager::update_frozen_scene, this, std::placeholders::_1)
        );

        // Publisher
        this->scene_objects_pub_ = this->create_publisher<cr_interfaces::msg::ObjectInfoArray>("cr/scene_objects", 10);

        RCLCPP_INFO(this->get_logger(), "ObjectStateManager is ready.");
    }


    void ObjectStateManager::update_scene_objects(const cr_interfaces::msg::ObjectInfoArray & object_array)
    {
        if (is_scene_frozen_)
        {
            RCLCPP_WARN(this->get_logger(), "Scene is frozen. Skipping update.");
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
                RCLCPP_DEBUG(this->get_logger(), "Object with id = %d already exists in the scene.", obj.id);
            }
        }
        // RCLCPP_INFO(this->get_logger(), "list of object_array: %d", object_array.objects.size());
        
        this->scene_objects_pub_->publish(object_array);
    }

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

    void ObjectStateManager::update_frozen_scene(const cr_interfaces::msg::FreezeScene & frozen_msg)
    {
        this->is_scene_frozen_ = frozen_msg.freeze;
    }

} // namespace vision
} // namespace cr
