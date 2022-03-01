import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
#from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Use composition for all image-processing nodes.
    
    Keeps overhead low since image data can – theoretically – reside in shared memory."""
    image_processing = ComposableNodeContainer(
            name = 'container',
            namespace = '',
            package = 'rclcpp_components',
            executable = 'component_container',
            composable_node_descriptions = [
                #ComposableNode(
                #    name = 'pylon_camera',
                #    namespace = '',
                #    package = 'pylon_instant_camera',
                #    plugin = 'pylon_instant_camera::PylonCameraNode',
                #    parameters = [
                #        {'camera_settings_pfs': get_package_share_directory('pylon_instant_camera')+'/config/rgb8.pfs'},
                #        {'camera_info_yaml': get_package_share_directory('pylon_instant_camera')+'/config/front_camera_calibration.yaml'}
                #    ],
                #    extra_arguments=[{'use_intra_process_comms': True}]
                #),
                ComposableNode(
                    name = 'address_printer',
                    namespace = '',
                    package = 'pylon_instant_camera',
                    plugin = 'AddressPrinter',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    name = 'publisher',
                    namespace = '',
                    package = 'pylon_instant_camera',
                    plugin = 'StaticBufferPublisher',
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output = 'screen'
    )
    return launch.LaunchDescription([image_processing])
