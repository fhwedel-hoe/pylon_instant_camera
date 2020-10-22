ROS node for access to Basler camera via Pylon CBaslerUsbInstantCamera API. 

Supports *low-latency, high-speed, arbitrary framerate, free-running* mode.

The official node does not support this mode, as discussed at these issues:

* https://github.com/magazino/pylon_camera/issues/25
* https://github.com/basler/pylon-ros-camera/issues/21
* https://github.com/basler/pylon-ros-camera/issues/28
* https://github.com/basler/pylon-ros-camera/issues/29

Used with a dart daA1280-54uc camera as installed on a BFFT/digitalwerk ADAS model car and the [FH Wedel Autonomous Driving ros_adas2019](https://github.com/FHW-AutonomousDriving/ros_adas2019) package.
