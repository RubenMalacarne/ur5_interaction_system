<!-- mainpage.md -->

# cr_bt_orchestrator

`cr_bt_orchestrator` is the core Behavior Tree control node of the system.  
It is responsible for receiving high-level workflow requests and coordinating the full pick-and-place execution flow using Behavior Trees.

This node integrates and manages multiple reusable BT nodes, custom logic conditions, service wrappers, subtrees to ensure safe, flexible, and modular orchestration of the entire robotic workflow.

## Behavior Tree Nodes

The `BehaviorTreeFactory` used in this package registers the following nodes:

### Action and Condition Nodes
- `FreezeScene`: calls a service to freeze the perception scene.
- `GetObjectInfo`: retrieves the ID and pose of the target object by its label.
- `IsStopRequested`: checks whether a stop request has been received.
- `IsPauseRequested`: checks whether a pause request has been received.
- `IsAreaSafe`: verifies the safety of the operational area.
- `WaitForTheGoAhead`: waits until the system is cleared to proceed.

### Logging Nodes (from `cr_bt_common`)
- `GuiLog`: publishes a log message to the GUI topic.
- `LogMessage`: logs an internal message using `rclcpp`.

## Integration

This package loads and integrates subtree definitions (`pick_subtree.xml` and `place_subtree.xml`) to encapsulate complex behaviors and modularize the workflow.

## Main Behavior Tree

The orchestrator loads the main BT XML: `pick_place_workflow.xml`.

## Authors
Sabrina Vinco and Ruben Malacarne