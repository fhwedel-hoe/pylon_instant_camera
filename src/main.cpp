#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <sensor_msgs/image_encodings.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#pragma GCC diagnostic pop

class PylonUSBCamera {
private:
    // This smart pointer will receive the grab result data.
    Pylon::CGrabResultPtr ptrGrabResult;
public:
    Pylon::CBaslerUsbInstantCamera * camera;
    int grab_timeout_ms = 1000;
    PylonUSBCamera() {
        Pylon::PylonInitialize();
        // Create an instant camera object with the camera device found first.
        camera = new Pylon::CBaslerUsbInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        camera->Open();
        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        camera->MaxNumBuffer = 5;
        camera->PixelFormat.SetValue(Basler_UsbCameraParams::PixelFormat_RGB8);
        // Start the grabbing.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    }
    ~PylonUSBCamera() {
        delete camera;
        Pylon::PylonTerminate(); 
    }
    Pylon::CGrabResultPtr & grab_frame() {
        if (camera->IsGrabbing()) {
            // Wait for an image and then retrieve it. A timeout of 1000 ms is used.
            // NOTE: performs an implicit copy. this is a memory/CPU bottleneck.
            // NOTE: Maybe using SetBufferFactory and an IBufferFactory pre-allocating sensor_msgs::msg::Image helps
            camera->RetrieveResult(grab_timeout_ms, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded()) {
                return ptrGrabResult;
            } else {
                throw std::runtime_error(std::string(ptrGrabResult->GetErrorDescription()));
            }
        } else {
            throw std::runtime_error("Camera is not grabbing.");
        }
    }
};

class PylonUSBCameraNode : public rclcpp::Node {
private:
    // the camera
    std::unique_ptr<PylonUSBCamera> camera;
    
    // camera info (for rectification data)
    sensor_msgs::msg::CameraInfo camera_info_msg;
    std::string frame_id = "pylon_camera";
    std::string camera_info_path;

    // publishers
    image_transport::CameraPublisher image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher;

    void parse_parameters() {
        // ros2 parameters
        frame_id = this->declare_parameter("frame_id", frame_id);
        camera_info_path = this->declare_parameter("camera_info_yaml", camera_info_path);
        // camera parameters
        camera->grab_timeout_ms = this->declare_parameter("grab_timeout", camera->grab_timeout_ms);
        RCLCPP_INFO(this->get_logger(), "grab_timeout is %d ms.", camera->grab_timeout_ms);
        int binning = this->declare_parameter("binning", 0);
        if (binning > 0) {
            camera->camera->BinningHorizontal.SetValue(binning);
            camera->camera->BinningVertical.SetValue(binning);
            RCLCPP_INFO(this->get_logger(), "binning set to %d.", binning);
        }
    }

public:
    PylonUSBCameraNode() : Node("pylon_usb_instant_camera"), camera(std::make_unique<PylonUSBCamera>()) {
        // set user-defined parameters
        parse_parameters();
        
        // stolen from https://github.com/clydemcqueen/opencv_cam/blob/master/src/opencv_cam_node.cpp
        std::string camera_name;
        if (!camera_info_path.empty() && camera_calibration_parsers::readCalibration(camera_info_path, camera_name, camera_info_msg)) {
            camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
        } else {
            RCLCPP_WARN(get_logger(), "camera_info was not loaded. image_proc will not perform rectification automatically.");
        }

        image_publisher = image_transport::create_camera_publisher(this, "image");
        
        // log some information
        RCLCPP_INFO(this->get_logger(), "Using device %s.", camera->camera->GetDeviceInfo().GetModelName().c_str());
        RCLCPP_INFO(this->get_logger(), "Expected frame-rate is %f.", camera->camera->ResultingFrameRate());
    }
    void grab_and_publish() {
        sensor_msgs::msg::Image img_msg = pylon_result_to_image_message(camera->grab_frame());
        img_msg.header.stamp = this->now();
        img_msg.header.frame_id = frame_id;
        camera_info_msg.header.stamp = img_msg.header.stamp;
        camera_info_msg.header.frame_id = img_msg.header.frame_id;
        image_publisher.publish(std::move(img_msg), camera_info_msg);
    }
private:
    sensor_msgs::msg::Image pylon_result_to_image_message(Pylon::CGrabResultPtr & ptrGrabResult) {
        // Access the image data.
        const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
        const size_t payloadSize = ptrGrabResult->GetPayloadSize();
        sensor_msgs::msg::Image img;
        img.width = ptrGrabResult->GetWidth();
        img.height = ptrGrabResult->GetHeight();
        if (ptrGrabResult->GetPixelType() != Pylon::EPixelType::PixelType_RGB8packed) {
            throw std::runtime_error("Captured image was not RGB8.");
        } else {
            img.encoding = sensor_msgs::image_encodings::RGB8;
            img.step = img.width * 3;
        }
        img.data.assign(pImageBuffer, pImageBuffer + payloadSize); // NOTE: explicit copy. this is a memory/CPU bottleneck.
        return img; // NOTE: this is NOT a bottle neck. copy-elision is strong in this one
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto camera_node = std::make_shared<PylonUSBCameraNode>();
    while(rclcpp::ok()) {
        camera_node->grab_and_publish();
        rclcpp::spin_some(camera_node);
    };
    return 0;
}
