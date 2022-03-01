Experimental ROS2 node for access to Basler camera via pylon CBaslerUniversalInstantCamera API. 

Supports *low-latency, high-speed, arbitrary framerate, free-running* mode.

Supports USB and GigE cameras.

Tested on Ubuntu 20.04 "Focal", ROS2 "foxy" and a Basler daA1280-54uc camera.

As of writing, the official driver does not support ROS2, yet. Check the status [here](https://github.com/basler/pylon-ros-camera/issues/58).

### Usage

#### Topic

If the camera provides pixels in bayer pattern (raw), the default topic name is `image_raw`.  
If the camera provides pixels in RGB (color) or MONO (grayscale), the default topic name is `image`.  

#### as a node

Start a node which publishes image data on topic `/image`.  
The camera will be opened using its default parameters:

    ros2 run pylon_instant_camera node

Start a node which publishes image data on topic `/pylon_camera/image_raw`.  
The camera will be opened and configured with the feature-set stored in `settings.pfs`.  
Camera calibration data will be loaded from `camera_calibration.yaml` and published at `/pylon_camera/camera_info` for image rectification.

    ros2 run pylon_instant_camera node --ros-args \
    -r __ns:=/pylon_camera \
    --param camera_settings_pfs:=settings_bayer.pfs \
    --param camera_info_yaml:=camera_calibration.yaml

#### within a Composit node

Launch file based on [this example](https://github.com/ros2/demos/blob/foxy/composition/launch/composition_demo.launch.py), with software debayer and camera image rectification:

    import launch
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode
    def generate_launch_description():
        """Use composition for all image-processing nodes.
        Keeps overhead low since image data can – theoretically – reside in shared memory."""
        image_processing = ComposableNodeContainer(
                name = 'container',
                namespace = 'pylon_camera_node',
                package = 'rclcpp_components',
                executable = 'component_container',
                composable_node_descriptions = [
                    ComposableNode(
                        name = 'pylon_camera',
                        namespace = 'pylon_camera_node',
                        package = 'pylon_instant_camera',
                        plugin = 'pylon_instant_camera::PylonCameraNode',
                        parameters = [{
                            'camera_settings_pfs': 'settings.pfs',
                            'camera_info_yaml': 'camera_calibration.yaml'
                            }]
                    ),
                    ComposableNode(
                        package='image_proc',
                        plugin='image_proc::DebayerNode',
                        name='debayer_node',
                        namespace='pylon_camera_node'
                    ),
                    ComposableNode(
                        package='image_proc',
                        plugin='image_proc::RectifyNode',
                        name='pylon_camera_rectify_color',
                        namespace='pylon_camera_node'
                    ),
                    ComposableNode(
                        package='image_proc',
                        plugin='image_proc::RectifyNode',
                        name='pylon_camera_rectify_mono',
                        namespace='pylon_camera_node',
                        remappings=[
                            ('image', 'image_mono')
                        ],
                    )
                ]
        )
        return launch.LaunchDescription([image_processing])

Viewing the unprocessed camera image is possible by manually loading the ShowImage tool like [this](https://docs.ros.org/en/foxy/Tutorials/Composition.html):

    ros2 component load /pylon_camera_node/container image_tools image_tools::ShowImage --node-namespace /pylon_camera_node


#### pylon Feature Stream (pfs)

The pylon Viewer included in the [pylon SDK](https://www.baslerweb.com/en/products/software/basler-pylon-camera-software-suite/pylon-sdks/) stores all the settings in one file of tab-separated key-value-pairs. Set the pixel type like this:

    # {05D8C294-F295-4dfb-9D01-096BD04049F4}
    # GenApi persistence file (version 3.1.0)
    PixelFormat	RGB8

With the pixel format RGB8, you do not need the DebayerNode and remappings mentioned in the example above.

### Known Issues

This warning can be emitted:

    Payload data has been discarded. Payload data can be discarded by the camera device if the available bandwidth is insufficient.

This is normal for USB connections and should only happen once after camera start.
