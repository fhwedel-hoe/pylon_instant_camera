#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#pragma GCC diagnostic pop
#ifdef CAMERA_INFO_MANAGER
#error "This was never adjusted for ros2 as I did not have a camera_info_manager at hand."
#include <camera_info_manager/camera_info_manager.hpp>
#endif

class PylonUSBCamera {
    private:
        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;
        Pylon::CBaslerUsbInstantCamera * camera;
        std::shared_ptr<rclcpp::Node> node;
    public:
        PylonUSBCamera(std::shared_ptr<rclcpp::Node> node) : node(node) {
            Pylon::PylonInitialize();
            // Create an instant camera object with the camera device found first.
            camera = new Pylon::CBaslerUsbInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
            camera->Open();
            // Print the model name of the camera.
            RCLCPP_INFO(node->get_logger(), "Using device %s", camera->GetDeviceInfo().GetModelName().c_str());
            // The parameter MaxNumBuffer can be used to control the count of buffers
            // allocated for grabbing. The default value of this parameter is 10.
            camera->MaxNumBuffer = 5;
            camera->PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_RGB8);
            //camera->Width.SetValue(640);
            //camera->Height.SetValue(480);
            //camera->OffsetX.SetValue(0);
            //camera->OffsetY.SetValue(0);
            //camera->BinningHorizontal.SetValue(2);
            //camera->BinningVertical.SetValue(2);
            // Start the grabbing.
            // The camera device is parameterized with a default configuration which
            // sets up free-running continuous acquisition.
            camera->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
            RCLCPP_INFO(node->get_logger(), "Expected frame-rate is %f.", camera->ResultingFrameRate());
        }
        ~PylonUSBCamera() {
            delete camera;
            Pylon::PylonTerminate(); 
        }
     Pylon::CGrabResultPtr & grab_frame() {
        if (camera->IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 1000 ms is used.
            camera->RetrieveResult(1000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                return ptrGrabResult;
            }
            else
            {
                throw std::runtime_error(
                    std::string(ptrGrabResult->GetErrorDescription())
                );
            }
        }
        else
        {
            throw std::runtime_error("Camera is not grabbing.");
        }
    }
};

sensor_msgs::msg::Image pylon_result_to_image_message(Pylon::CGrabResultPtr & ptrGrabResult) {
    // Access the image data.
    const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
    const size_t payloadSize = ptrGrabResult->GetPayloadSize();
    sensor_msgs::msg::Image img;
    img.width = ptrGrabResult->GetWidth();
    img.height = ptrGrabResult->GetHeight();
    img.encoding = sensor_msgs::image_encodings::RGB8; // TODO: check if this actually matches the camera settings
    img.step = img.width * 3; // TODO: see above
    img.data.assign(pImageBuffer, pImageBuffer + payloadSize);
    return img;
    }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("pylon_usbinstantcamera");
    
    // set quality of service as demonstrated at https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp
    rclcpp::QoS qos(1); // keep latest
    // qos = qos.best_effort(); // when using best_effort, no images are transferred
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher = node->create_publisher<sensor_msgs::msg::Image>("image", qos);
#ifdef CAMERA_INFO_MANAGER
    #error "This was never adjusted for ros2 as I did not have a camera_info_manager at hand."
    camera_info_manager::CameraInfoManager cim(nh);
    std::string camera_info_url;
    if (nh.getParam("camera_info_url", camera_info_url))
    {
        cim.loadCameraInfo(camera_info_url);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "camera_info_url not supplied in configuration. Camera info will be unavailable. Rectification is not possible.");
    }
    sensor_msgs::CameraInfo cam_info = cim.getCameraInfo();
#endif
    PylonUSBCamera camera(node);
    while(rclcpp::ok()) {
        sensor_msgs::msg::Image img = pylon_result_to_image_message(camera.grab_frame());
        img.header.stamp = node->now(); 
#ifdef CAMERA_INFO_MANAGER
        cam_info.header.stamp = img.header.stamp;
        publisher->publish(img, cam_info);
#else
        publisher->publish(img);
#endif
        rclcpp::spin_some(node);
    };
    return 0;
}
