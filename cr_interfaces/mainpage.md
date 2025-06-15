<!-- mainpage.md -->

# cr_interfaces

`cr_interfaces` is a ROS 2 package for all interfaces (action, msgs and server) of the project.

there are only 3 folders for msgs, actions and services about our ros2 workspace, no launch file or other script.

The structure of the msgs is the following:


cr_interfaces
|──action
│   ├── ExecuteWorkflow.action
│   ├── Pick.action
│   └── Place.action
│
├── msg
│   ├── DetectedObjects.msg
│   ├── DetectedSurfaces.msg
│   ├── FreezeScene.msg
│   ├── InferenceResult.msg
│   ├── Log.msg
│   ├── ObjectCount.msg
│   ├── ObjectDetectionBox.msg
│   ├── ObjectDetectionResult.msg
│   ├── ObjectInfo.msg
│   ├── ObjectInfoArray.msg
│   ├── PoseKeypoint.msg
│   ├── PoseResult.msg
│   ├── SegmentationResult.msg
│   ├── YoloV8Inference.msg
│   └── YoloV8Segmentation.msg
│
└── srv
    ├── AllowCollision.srv
    ├── AttachObject.srv
    ├── FreezeScene.srv
    └── GetObjectInfo.srv

## Author
Sabrina Vinco and Ruben Malacarne