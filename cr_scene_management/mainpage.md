<!-- mainpage.md -->

# cr_scene_management

The `cr_scene_management` package handles the dynamic and static management of the MoveIt planning scene.

It provides nodes to publish static elements, and to update the scene in real-time based on perception and interaction events.

## Node list
- `planning_scene_modifier`: manages dynamic updates (spawn, attach, collisions) 
- `static_scene_publisher`: publishes static objects once at startup. 

## Authors
Ruben Malacarne and Sabrina Vinco