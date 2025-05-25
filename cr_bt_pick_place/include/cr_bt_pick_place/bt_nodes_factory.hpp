#ifndef CR_BT_PICK_PLACE__BT_NODES_FACTORY_HPP_
#define CR_BT_PICK_PLACE__BT_NODES_FACTORY_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace cr::bt::pick_place
{
    void registerNodes(BT::BehaviorTreeFactory& factory, 
                       rclcpp::Node::SharedPtr node);
                       
    void registerSubtrees(BT::BehaviorTreeFactory& factory);
}

#endif
