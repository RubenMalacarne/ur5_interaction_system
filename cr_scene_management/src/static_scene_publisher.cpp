#include "cr_scene_management/static_scene_publisher.hpp"
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <shape_msgs/msg/mesh.hpp>
#include <geometric_shapes/shape_operations.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cr_scene_management/scene_manager.hpp>

namespace cr {
namespace scene_management {

    /**
     * @brief Constructor for the StaticScenePublisher node.
     * Initializes the planning scene publisher and starts a timer to periodically publish a static planning scene.
     * 
     * @param options Optional node configuration parameters.
     */
    StaticScenePublisher::StaticScenePublisher(const rclcpp::NodeOptions& options)
    : Node("static_scene_publisher", options)
    {
        planning_scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

        timer_ = create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&StaticScenePublisher::publishStaticScene, this));

        RCLCPP_INFO(get_logger(), "StaticScenePublisher inizializzato.");
    }

    /**
     * @brief Publishes a static collision object (e.g., a table) to the MoveIt planning scene.
     * 
     * This method:
     * - Retrieves the current PlanningSceneMonitor.
     * - Loads a mesh for a table object.
     * - Adds the object as a collision object to the scene.
     * - Updates the Allowed Collision Matrix (ACM) to allow collisions with a specific link.
     * - Publishes the planning scene diff.
     */
    void StaticScenePublisher::publishStaticScene()
    {
        // 1) Ottengo il PlanningSceneMonitor dal SceneManager
        auto& manager = cr::scene_management::SceneManager::instance(shared_from_this());
        auto psm = manager.getPlanningSceneMonitor();

        if (!psm || !psm->getPlanningScene()) {
            RCLCPP_ERROR(get_logger(), "PlanningSceneMonitor non disponibile!");
            return;
        }

        // 2) Creare un collision object per un tavolo
        moveit_msgs::msg::CollisionObject table;
        table.id = "table";
        table.header.frame_id = "world";
        table.operation = table.ADD;

        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;

        // Carico una mesh
        std::string pkg_share = ament_index_cpp::get_package_share_directory("cr_hw_configuration");
        std::string mesh_path = pkg_share + "/meshes/lab_table_mesh.stl";
        shapes::Mesh* shape_mesh = shapes::createMeshFromResource("file://" + mesh_path);
        if (!shape_mesh) {
            RCLCPP_ERROR(get_logger(), "Impossibile caricare la mesh %s", mesh_path.c_str());
            return;
        }
        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(shape_mesh, shape_msg);
        shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

        table.meshes.push_back(mesh_msg);
        table.mesh_poses.push_back(table_pose);

        // 3) Creiamo un PlanningScene diff
        moveit_msgs::msg::PlanningScene scene_msg;
        scene_msg.is_diff = true;
        scene_msg.world.collision_objects.push_back(table);

        // Creiamo un lock in scrittura sulla planning scene
        planning_scene_monitor::LockedPlanningSceneRW locked_scene(psm);
        collision_detection::AllowedCollisionMatrix& acm =
            locked_scene->getAllowedCollisionMatrixNonConst();

        acm.setEntry("base_link_inertia", "table", true);

        // Convertiamo l'ACM in un msg e lo mettiamo in scene_msg
        acm.getMessage(scene_msg.allowed_collision_matrix);

        // 5) Pubblicazione diff
        planning_scene_pub_->publish(scene_msg);

        RCLCPP_INFO(get_logger(), "Pubblicata scena statica (tavolo).");
        timer_->cancel(); // se vuoi pubblicarlo solo una volta
    }

}  // namespace scene_management
}  // namespace cr

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cr::scene_management::StaticScenePublisher)
