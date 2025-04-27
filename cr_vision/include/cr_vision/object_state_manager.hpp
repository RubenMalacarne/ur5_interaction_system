#include <rclcpp/rclcpp.hpp>
#include <cr_interfaces/msg/object_info_array.hpp>
#include <cr_interfaces/srv/get_object_info.hpp>
#include <cr_interfaces/msg/freeze_scene.hpp>
#include <unordered_map>

namespace cr {
namespace vision{

    class ObjectStateManager : public rclcpp::Node
    {
    public:

        explicit ObjectStateManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:

        bool is_scene_frozen_ = false;

        // Mappa per mantenere lo stato dei vari oggetti
        std::unordered_map<uint8_t, cr_interfaces::msg::ObjectInfo> scene_objects_;

        rclcpp::Publisher<cr_interfaces::msg::ObjectInfoArray>::SharedPtr scene_objects_pub_;
        rclcpp::Service<cr_interfaces::srv::GetObjectInfo>::SharedPtr get_object_info_srv_;
        rclcpp::Subscription<cr_interfaces::msg::ObjectInfoArray>::SharedPtr object_info_array_sub_;
        rclcpp::Subscription<cr_interfaces::msg::FreezeScene>::SharedPtr freeze_scene_sub_;

        // Callback per restituire le informazioni di uno specifico oggetto
        void give_object_info(const std::shared_ptr<cr_interfaces::srv::GetObjectInfo::Request> request,
            std::shared_ptr<cr_interfaces::srv::GetObjectInfo::Response> response);

        // Callback per aggiornare lo stato interno della scena e pubblicare su topic gli oggetti (solo nel caso in cui la scena non sia congelata)
        void update_scene_objects(const cr_interfaces::msg::ObjectInfoArray & object_array);

        // Callback per aggiornare il booleano 'is_scene_frozen_'
        void update_frozen_scene(const cr_interfaces::msg::FreezeScene & frozen_msg);

    };

} // namespace vision
} // namespace cr