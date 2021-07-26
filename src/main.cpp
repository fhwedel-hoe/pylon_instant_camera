#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <sensor_msgs/image_encodings.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/DeviceInfo.h>
#pragma GCC diagnostic pop

namespace pylon_usb_instant_camera {

// this BufferFactory pre-allocates sensor_msgs::msg::Image memory
// so the Basler Pylon SDK can write into the data vectors directly
class ImageBufferFactory : public Pylon::IBufferFactory {
public:
    ~ImageBufferFactory() {
    }
    virtual void AllocateBuffer(size_t bufferSize, void **pCreatedBuffer, intptr_t &bufferContext) {
        sensor_msgs::msg::Image * image_message_ptr = new sensor_msgs::msg::Image();
        image_message_ptr->data.resize(bufferSize);
        *pCreatedBuffer = image_message_ptr->data.data();
        bufferContext = reinterpret_cast<intptr_t>(image_message_ptr);
    }
    virtual void FreeBuffer(void *, intptr_t bufferContext) {
        delete ReinterpretBufferContext(bufferContext);
    }
    virtual void DestroyBufferFactory() {
    }
    static sensor_msgs::msg::Image * ReinterpretBufferContext(intptr_t bufferContext) {
        return reinterpret_cast<sensor_msgs::msg::Image *>(bufferContext);
    }
};

class PylonUSBCamera {
private:
    // This smart pointer will receive the grab result data.
    Pylon::CGrabResultPtr ptrGrabResult;
public:
    Pylon::CBaslerUniversalInstantCamera * camera;
    int grab_timeout_ms = 1000;
    PylonUSBCamera(std::string full_name, std::string user_defined_name, std::string ip_address) {
        Pylon::PylonInitialize();
        try {
			// Create an instant camera object with the camera device found first or found by specified parameters.
            if (full_name == "not_specified" && user_defined_name == "not_specified" && ip_address == "not_specified") {
                camera = new Pylon::CBaslerUniversalInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
            } else {
                Pylon::CDeviceInfo di;
                if (full_name != "not_specified") di.SetFullName(full_name.c_str());
                if (user_defined_name != "not_specified") di.SetUserDefinedName(user_defined_name.c_str());
                if (ip_address != "not_specified") di.SetIpAddress(ip_address.c_str());
                camera = new Pylon::CBaslerUniversalInstantCamera(Pylon::CTlFactory::GetInstance().CreateDevice(di));
            }
			camera->Open();
			// provide Pylon with sensor_msgs::msg::Image buffers
			camera->SetBufferFactory(new ImageBufferFactory());
			// The parameter MaxNumBuffer can be used to control the count of buffers
			// allocated for grabbing. The default value of this parameter is 10.
			camera->MaxNumBuffer = 5;
			camera->PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG8);
			// Start the grabbing.
			// The camera device is parameterized with a default configuration which
			// sets up free-running continuous acquisition.
			camera->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
		} catch (const GenICam_3_1_Basler_pylon::RuntimeException & e) {
			std::cerr << e.what() << std::endl;
			throw e;
		} catch (const GenICam_3_1_Basler_pylon::AccessException & e) {
			std::cerr << e.what() << std::endl;
			throw e;
		}
    }
    ~PylonUSBCamera() {
        delete camera;
        Pylon::PylonTerminate(); 
    }
    Pylon::CGrabResultPtr & grab_frame() {
        if (camera->IsGrabbing()) {
            // Wait for an image and then retrieve it.
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
    std::string camera_info_path = "camera_calibration.yaml";
    std::string full_name = "not_specified";
    std::string user_defined_name = "not_specified";
    std::string ip_address = "not_specified";

    // publisher
    image_transport::CameraPublisher image_publisher;
    
    // background thread
    std::unique_ptr<std::thread> grabbing_thread;

    void parse_parameters() {
        // ros2 parameters
        frame_id = this->declare_parameter("frame_id", frame_id);
        camera_info_path = this->declare_parameter("camera_info_yaml", camera_info_path);
        // camera parameters
        camera->grab_timeout_ms = this->declare_parameter("grab_timeout", camera->grab_timeout_ms);
        RCLCPP_INFO(this->get_logger(), "grab_timeout is %d ms.", camera->grab_timeout_ms);
        // TODO: load all parameters from pfs file
        int binning = this->declare_parameter("binning", 0);
        if (binning > 0) {
            camera->camera->BinningHorizontal.SetValue(binning);
            camera->camera->BinningVertical.SetValue(binning);
            RCLCPP_INFO(this->get_logger(), "binning set to %d.", binning);
        }
    }

public:
    PylonUSBCameraNode(const rclcpp::NodeOptions & options) : 
        Node("pylon_usb_instant_camera", options)
    {
        // parse device ID and IP address parameters to construct the camera object
        full_name = this->declare_parameter("full_name", full_name);
        user_defined_name = this->declare_parameter("user_defined_name", user_defined_name);
        ip_address = this->declare_parameter("ip_address", ip_address);
        RCLCPP_INFO(this->get_logger(), "Constructing camera object for full name [%s], user defined name [%s], IP address [%s].", full_name.c_str(), user_defined_name.c_str(), ip_address.c_str());
        camera = std::make_unique<PylonUSBCamera>(full_name, user_defined_name, ip_address);

        // set user-defined parameters
        parse_parameters();

        // stolen from https://github.com/clydemcqueen/opencv_cam/blob/master/src/opencv_cam_node.cpp
        std::string camera_name;
        if (!camera_calibration_parsers::readCalibration(camera_info_path, camera_name, camera_info_msg)) {
            RCLCPP_WARN(get_logger(), "camera_info was not loaded. image_proc will not perform rectification automatically.");
        }

        image_publisher = image_transport::create_camera_publisher(this, "image");
        
        // log some information
        RCLCPP_INFO(this->get_logger(), "Using device [%s] with full name [%s] and user defined name [%s].", camera->camera->GetDeviceInfo().GetModelName().c_str(), camera->camera->GetDeviceInfo().GetFullName().c_str(), camera->camera->GetDeviceInfo().GetUserDefinedName().c_str());
        //RCLCPP_INFO(this->get_logger(), "Expected frame-rate is %f.", camera->camera->ResultingFrameRate());
        
        grabbing_thread = std::make_unique<std::thread>([this](){
			// have the main loop in a thread since blocking functions and rclcpp::spin() are mutually exclusive
            while(rclcpp::ok()) {
                try {
                    this->grab_and_publish();
                } catch (const GenICam_3_1_Basler_pylon::TimeoutException & e) {
                    RCLCPP_WARN(get_logger(), "Timeout while grabbing.");
                }
            }
        });
    }
    void grab_and_publish() {
        sensor_msgs::msg::Image * img_msg = pylon_result_to_image_message(camera->grab_frame());
        img_msg->header.stamp = this->now();
        img_msg->header.frame_id = frame_id;
        camera_info_msg.header.stamp = img_msg->header.stamp;
        camera_info_msg.header.frame_id = img_msg->header.frame_id;
        image_publisher.publish(*img_msg, camera_info_msg);
    }
private:
    sensor_msgs::msg::Image * pylon_result_to_image_message(Pylon::CGrabResultPtr & ptrGrabResult) {
        sensor_msgs::msg::Image * img = ImageBufferFactory::ReinterpretBufferContext(ptrGrabResult->GetBufferContext());
        img->width = ptrGrabResult->GetWidth();
        img->height = ptrGrabResult->GetHeight();
        size_t stride;
        ptrGrabResult->GetStride(stride);
        RCLCPP_DEBUG(this->get_logger(), "Got image %ix%i, stride %i, size %i bytes, publishing.", img->width, img->height, stride, ptrGrabResult->GetBufferSize());
        if (ptrGrabResult->GetPixelType() != Pylon::EPixelType::PixelType_BayerRG8) {
            throw std::runtime_error("Captured image was not Bayer RG 8.");
        } else {
            img->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
            img->step = stride;
        }
        return img; // NOTE: this is NOT a bottleneck. copy-elision is strong in this one.
    }
};

} // end namespace pylon_usb_instant_camera

RCLCPP_COMPONENTS_REGISTER_NODE(pylon_usb_instant_camera::PylonUSBCameraNode)
