<!-- mainpage.md -->

# cr_bt_pick_place

The `cr_bt_pick_place` package provides Behavior Tree nodes and reusable subtrees for orchestrating pick-and-place tasks in a modular way.

It integrates with `cr_motion_core` for robot control and exposes service client wrappers to manipulate the planning scene and gripper behavior at runtime.

## Behavior Tree Nodes

### Arm Motion Nodes
- `ArmHorizontalMove`: moves the robot arm in the XY plane, keeping the Z coordinate fixed.

- `ArmVerticalMove`: moves the robot arm up or down along the Z axis.

- `GoHome`: moves the robot arm to a predefined "home" pose.

### Gripper Node

- `SetGripper`: opens or closes the gripper to a specified joint value (in radians).

### Scene Interaction Nodes

- `SetCollisionAllowed`: enables or disables collision checking for a specific object using the `/allow_collision` service.

- `SetObjectAttached`: attaches or detaches an object to/from the end-effector using the `/attach_object` service.

## Subtrees

Two reusable subtrees are provided and registered dynamically:

- `pick_subtree.xml`: encapsulates the full pick sequence.
- `place_subtree.xml`: encapsulates the place sequence.

These XML files are placed in the `bt_xml/` directory within the package.

## Integration

Nodes must be registered using the utility function:

```cpp
cr::bt::pick_place::registerNodes(factory, node);
```

And subtrees can be registered via:

```cpp
cr::bt::pick_place::registerSubtrees(factory);
```

## Authors
Sabrina Vinco and Ruben Malacarne