/**
 * @file bt_nodes_factory.hpp
 * @brief Registration utilities for pick-and-place Behavior Tree nodes and subtrees.
 *
 * This file defines utility functions to register the custom BT nodes and XML subtrees
 * used in the pick-and-place pipeline.
 */

#ifndef CR_BT_PICK_PLACE__BT_NODES_FACTORY_HPP_
#define CR_BT_PICK_PLACE__BT_NODES_FACTORY_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace cr::bt::pick_place
{

    /**
     * @brief Registers all custom nodes required for the pick-and-place BT.
     *
     * Nodes registered:
     * - ArmHorizontalMove
     * - ArmVerticalMove
     * - SetGripper
     * - GoHome
     * - SetCollisionAllowed (service-based)
     * - SetObjectAttached (service-based)
     *
     * @param factory Reference to the BT factory to register nodes into.
     * @param node Shared pointer to the ROS 2 node used for logging and services.
     */
    void registerNodes(BT::BehaviorTreeFactory &factory,
                       rclcpp::Node::SharedPtr node);

    /**
     * @brief Registers reusable subtree XML definitions for pick and place.
     *
     * This function loads subtree files from the `bt_xml/` directory
     * within the `cr_bt_pick_place` package share directory.
     *
     * Subtrees:
     * - `pick_subtree.xml`
     * - `place_subtree.xml`
     *
     * @param factory BehaviorTreeFactory instance to register the subtrees into.
     */
    void registerSubtrees(BT::BehaviorTreeFactory &factory);

} // namespace cr::bt::pick_place

#endif // CR_BT_PICK_PLACE__BT_NODES_FACTORY_HPP_
