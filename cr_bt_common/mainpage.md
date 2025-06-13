<!-- mainpage.md -->

# cr_bt_common

`cr_bt_common` provides a set of reusable Behavior Tree nodes, which encapsulate common utilities and logging functionality that can be shared across multiple behavior trees.

## BT Nodes

- `CheckBlackboardKeyNode`: condition node that checks whether a specified key exists in the blackboard. 

- `GuiLog`: action node that publishes a structured log message to a GUI-compatible ROS topic.

- `LogMessageNode`: action node that logs a string message to the ROS console. Useful for quick debugging or runtime status messages during tree execution.

## Authors
Ruben Malacarne and Sabrina Vinco