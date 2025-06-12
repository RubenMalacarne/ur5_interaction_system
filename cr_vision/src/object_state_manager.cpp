// Node Responsibilities:
// 1. Depending on whether the scene should be frozen or not, it manages sending updates to the MoveIt planning_scene
// 2. Internally maintains a status table of the identified objects
// 3. Provides a service to request information about a specific object

#include "cr_vision/object_state_manager.hpp"

namespace cr
{
    namespace vision
    {

        ObjectStateManager::ObjectStateManager(const rclcpp::NodeOptions &options)
            : Node("object_state_manager", options)
        {
            RCLCPP_INFO(this->get_logger(), "Starting ObjectStateManager...");

            // Services
            this->get_object_info_srv_ = this->create_service<cr_interfaces::srv::GetObjectInfo>(
                "cr/get_object_info",
                std::bind(&ObjectStateManager::give_object_info, this, std::placeholders::_1, std::placeholders::_2));

            this->freeze_scene_srv_ = this->create_service<cr_interfaces::srv::FreezeScene>(
                "cr/freeze_scene",
                std::bind(&ObjectStateManager::handle_freeze_scene_request, this, std::placeholders::_1, std::placeholders::_2));

            // Subscriptions
            this->object_info_array_sub_ = this->create_subscription<cr_interfaces::msg::ObjectInfoArray>(
                "cr_vision/detected_objects",
                10,
                std::bind(&ObjectStateManager::update_scene_objects, this, std::placeholders::_1));

            // Publisher
            this->scene_objects_pub_ = this->create_publisher<cr_interfaces::msg::ObjectInfoArray>("cr/scene_objects", 10);

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
                // Trovato almeno un oggetto con la label richiesta
                response->success = true;
                response->object_info = scene_objects_[*min_id];
                RCLCPP_INFO(
                    this->get_logger(),
                    "Object with label '%s' found (id = %d), returning info.",
                    target_label.c_str(),
                    *min_id);
            }
            else
            {
                // Nessun oggetto con quella label
                response->success = false;
                RCLCPP_WARN(
                    this->get_logger(),
                    "No object with label '%s' found in the scene.",
                    target_label.c_str());
            }
        }

        void ObjectStateManager::handle_freeze_scene_request(const std::shared_ptr<cr_interfaces::srv::FreezeScene::Request> request,
                                                             std::shared_ptr<cr_interfaces::srv::FreezeScene::Response> response)
        {
            this->is_scene_frozen_ = request->freeze;

            if (request->freeze)
            {
                RCLCPP_INFO(this->get_logger(), "Planning scene has been frozen.");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Planning scene has been unfrozen.");
            }

            response->success = true;
        }

    } // namespace vision
} // namespace cr
