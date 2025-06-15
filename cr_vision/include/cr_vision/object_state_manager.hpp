/**
 * @file object_state_manager.hpp
 * @brief Declares ObjectStateManager node for managing object state in the scene.
 *
 * This node keeps track of known objects in the scene.
 * It listens to updated object lists, stores their state,
 * and can provide info about specific objects.
 * It also supports freezing the scene (ignoring new updates).
 *
 * Services:
 *  - /cr/get_object_info: returns info about a specific object
 *  - /cr/freeze_scene: freezes/unfreezes scene updates
 *
 * Publisher:
 *  - /cr/scene_objects: current internal state of all tracked objects
 *
 * Subscriber:
 *  - /cr/object_info_array: incoming updates
 *
 * @ingroup cr_vision_nodes
 */

#ifndef CR_VISION_OBJECT_STATE_MANAGER_HPP_
#define CR_VISION_OBJECT_STATE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <cr_interfaces/msg/object_info_array.hpp>
#include <cr_interfaces/srv/get_object_info.hpp>
#include <cr_interfaces/srv/freeze_scene.hpp>
#include <unordered_map>

namespace cr::vision
{
    /**
     * @brief Manages the internal state of detected objects.
     *
     * Stores the last known pose and label of each object, provides access
     * to individual object info, and optionally freezes the scene to prevent updates.
     *
     * @ingroup cr_vision_nodes
     */
    class ObjectStateManager : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor.
         *
         * Initializes services, subscriber and publisher.
         * Sets up the internal scene state.
         *
         * @param options NodeOptions for ROS2 configuration.
         */
        explicit ObjectStateManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        /**
         * @brief Default destructor.
         */
        ~ObjectStateManager() override = default;

    private:
        /// Whether the scene is frozen (updates are ignored)
        bool is_scene_frozen_ = false;

        /// Map storing the current state of all objects, indexed by ID
        std::unordered_map<uint8_t, cr_interfaces::msg::ObjectInfo> scene_objects_;

        /// Publishes the internal object state
        rclcpp::Publisher<cr_interfaces::msg::ObjectInfoArray>::SharedPtr scene_objects_pub_;

        /// Service to get info about a specific object
        rclcpp::Service<cr_interfaces::srv::GetObjectInfo>::SharedPtr get_object_info_srv_;

        /// Service to freeze/unfreeze scene updates
        rclcpp::Service<cr_interfaces::srv::FreezeScene>::SharedPtr freeze_scene_srv_;

        /// Subscriber for incoming object info updates
        rclcpp::Subscription<cr_interfaces::msg::ObjectInfoArray>::SharedPtr object_info_array_sub_;

        /**
         * @brief Service callback to return the object with a given label.
         *
         * Among all stored objects, returns the one with the given label and the lowest ID.
         *
         * @param request Contains the target label.
         * @param response Filled with the matching object (if found).
         */
        void give_object_info(const std::shared_ptr<cr_interfaces::srv::GetObjectInfo::Request> request,
                              std::shared_ptr<cr_interfaces::srv::GetObjectInfo::Response> response);

        /**
         * @brief Service callback to freeze or unfreeze the scene.
         *
         * When frozen, the internal state will not be updated even if new messages arrive.
         *
         * @param request Contains the freeze/unfreeze command.
         * @param response Indicates if the request was accepted.
         */
        void handle_freeze_scene_request(const std::shared_ptr<cr_interfaces::srv::FreezeScene::Request> request,
                                         std::shared_ptr<cr_interfaces::srv::FreezeScene::Response> response);

        /**
         * @brief Callback for new object info messages.
         *
         * If the scene is not frozen, updates the internal map and republishes the full scene.
         *
         * @param object_array The incoming array of detected objects.
         */
        void update_scene_objects(const cr_interfaces::msg::ObjectInfoArray &object_array);
    };

} // namespace cr::vision

#endif // CR_VISION_OBJECT_STATE_MANAGER_HPP_