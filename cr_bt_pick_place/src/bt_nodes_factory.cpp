#include "cr_bt_pick_place/bt_nodes_factory.hpp"
#include "cr_bt_pick_place/arm_horizontal_move_node.hpp"
#include "cr_bt_pick_place/arm_vertical_move_node.hpp"
#include "cr_bt_pick_place/go_home_node.hpp"
#include "cr_bt_pick_place/set_gripper_node.hpp"
#include "cr_bt_pick_place/set_collision_allowed_node.hpp"
#include "cr_bt_pick_place/set_object_attached_node.hpp"

#include <behaviortree_ros2/bt_service_node.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace cr::bt::pick_place
{
    void registerNodes(BT::BehaviorTreeFactory& factory, rclcpp::Node::SharedPtr node)
    {
        BT::RosNodeParams default_ros_params;
        default_ros_params.nh = node;
        
        factory.registerNodeType<nodes::ArmHorizontalMoveNode>("ArmHorizontalMove");
        factory.registerNodeType<nodes::ArmVerticalMoveNode>("ArmVerticalMove");
        factory.registerNodeType<nodes::SetGripperNode>("SetGripper");
        factory.registerNodeType<nodes::GoHomeNode>("GoHome");
        
        BT::RosNodeParams collision_params = default_ros_params;
        collision_params.default_port_value = "/allow_collision";
        factory.registerNodeType<nodes::SetCollisionAllowedNode>("SetCollisionAllowed", collision_params);

        BT::RosNodeParams attach_params = default_ros_params;
        attach_params.default_port_value = "/attach_object";
        factory.registerNodeType<nodes::SetObjectAttachedNode>("SetObjectAttached", attach_params);
    }
    
    void registerSubtrees(BT::BehaviorTreeFactory& factory)
    {
        std::string current_package_path = ament_index_cpp::get_package_share_directory("cr_bt_pick_place");

        const std::string pick_xml_path = current_package_path + "/bt_xml/pick_subtree.xml";
        const std::string place_xml_path = current_package_path + "/bt_xml/place_subtree.xml";
        
        if (std::filesystem::exists(pick_xml_path)) {
            factory.registerBehaviorTreeFromFile(pick_xml_path);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("cr_pick_place"), 
                        "Pick subtree XML file not found at: %s", pick_xml_path.c_str());
        }
        
        if (std::filesystem::exists(place_xml_path)) {
            factory.registerBehaviorTreeFromFile(place_xml_path);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("cr_pick_place"), 
                        "Place subtree XML file not found at: %s", place_xml_path.c_str());
        }
    }
}
