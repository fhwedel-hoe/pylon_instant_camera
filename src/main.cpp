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

namespace pylon_instant_camera {

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

class PylonCamera {
private:
    // This smart pointer will receive the grab result data.
    Pylon::CGrabResultPtr ptrGrabResult;
public:
    Pylon::CBaslerUniversalInstantCamera * camera;
    int grab_timeout_ms = 1000;
    PylonCamera(const std::string & full_name, const std::string & user_defined_name, const std::string & ip_address, const int serial_number) {
        Pylon::PylonInitialize();
        // Create an instant camera object with the camera device found first or found by specified parameters.
        if (!full_name.empty() && !user_defined_name.empty() && !ip_address.empty() && serial_number == 0) {
            camera = new Pylon::CBaslerUniversalInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        } else {
            Pylon::CDeviceInfo di;
            if (!full_name.empty()) { 
                di.SetFullName(full_name.c_str()); 
            }
            if (!user_defined_name.empty()) { 
                di.SetUserDefinedName(user_defined_name.c_str()); 
            }
            if (!ip_address.empty()) { 
                di.SetIpAddress(ip_address.c_str()); 
            }
            if (serial_number != 0) {
                di.SetSerialNumber(std::to_string(serial_number).c_str());
            }
            camera = new Pylon::CBaslerUniversalInstantCamera(Pylon::CTlFactory::GetInstance().CreateDevice(di));
        }
        camera->Open();
        // provide Pylon with sensor_msgs::msg::Image buffers
        camera->SetBufferFactory(new ImageBufferFactory());
        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        camera->MaxNumBuffer = 5;
    }
    ~PylonCamera() {
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

class PylonCameraNode : public rclcpp::Node {
private:
    // the camera
    std::unique_ptr<PylonCamera> camera;
    
    // camera info (for rectification data)
    sensor_msgs::msg::CameraInfo camera_info_msg;
    std::string frame_id = "pylon_camera";
    std::string camera_info_path = "camera_calibration.yaml";
    std::string camera_settings_path;
    
    // camera info (for connection, esp. GigE)
    std::string full_name;
    std::string user_defined_name;
    std::string ip_address;
    int serial_number; // should be a string, but https://answers.ros.org/question/236326/running-ros-node-with-a-numeric-parameter-passed-as-string-doesnt-work/ is still a problem

    // publisher
    image_transport::CameraPublisher image_publisher;
    
    // background thread
    std::unique_ptr<std::thread> grabbing_thread;

public:
    PylonCameraNode(const rclcpp::NodeOptions & options) : 
        Node("pylon_instant_camera", options)
    {
        // ros2 parameters
        frame_id = this->declare_parameter("frame_id", frame_id);
        camera_info_path = this->declare_parameter("camera_info_yaml", camera_info_path);

        // parse device ID and IP address parameters to construct the camera object
        full_name = this->declare_parameter("full_name", full_name);
        user_defined_name = this->declare_parameter("user_defined_name", user_defined_name);
        ip_address = this->declare_parameter("ip_address", ip_address);
        serial_number = this->declare_parameter("serial_number", serial_number);

        RCLCPP_INFO(
            this->get_logger(), 
            "Constructing camera object for full name [%s], user defined name [%s], IP address [%s], serial number [%d].", 
            full_name.c_str(), user_defined_name.c_str(), ip_address.c_str(), serial_number
        );
        try {
            camera = std::make_unique<PylonCamera>(full_name, user_defined_name, ip_address, serial_number);
        } catch (const GenICam::RuntimeException & e) {
            RCLCPP_ERROR(get_logger(), "Exception in PylonCamera constructor: %s", e.what());
            throw e;
        }

        // camera parameters
        camera_settings_path = this->declare_parameter("camera_settings_pfs", camera_settings_path);
        if (!camera_settings_path.empty()) {
            try {
                Pylon::CFeaturePersistence::Load(camera_settings_path.c_str(), &camera->camera->GetNodeMap(), true);
            } catch (GenICam::RuntimeException &e) {
                RCLCPP_WARN(this->get_logger(), 
                    "Loading camera settings from PFS file %s failed: %s", 
                    camera_settings_path.c_str(), e.what()
                );
            }
        }
        
        camera->grab_timeout_ms = this->declare_parameter("grab_timeout", camera->grab_timeout_ms);
        RCLCPP_INFO(this->get_logger(), "grab_timeout is %d ms.", camera->grab_timeout_ms);

        // stolen from https://github.com/clydemcqueen/opencv_cam/blob/master/src/opencv_cam_node.cpp
        std::string camera_name;
        if (!camera_calibration_parsers::readCalibration(camera_info_path, camera_name, camera_info_msg)) {
            RCLCPP_WARN(get_logger(), "camera_info was not loaded. image_proc will not perform rectification automatically.");
        }
        
        std::string publish_topic = "image";
        Pylon::CPixelTypeMapper pixel_type_mapper(&camera->camera->PixelFormat);
        const Pylon::EPixelType pixel_type = pixel_type_mapper.GetPylonPixelTypeFromNodeValue(camera->camera->PixelFormat.GetIntValue());
        if (Pylon::IsBayer(pixel_type)) {
            // if PixelType is bayer, topic should be image_raw, according to
            // https://github.com/ros-perception/image_pipeline/blob/ros2/image_proc/src/debayer.cpp
            publish_topic = "image_raw";
            RCLCPP_INFO(this->get_logger(), "Input pixel type is bayer.");
        }
        RCLCPP_INFO(this->get_logger(), "Output topic name is [%s].", publish_topic.c_str());
        image_publisher = image_transport::create_camera_publisher(this, publish_topic.c_str());
        
        // log some information
        RCLCPP_INFO(
            this->get_logger(), 
            "Using device [%s] with full name [%s], user defined name [%s] and serial number [%s].", 
            camera->camera->GetDeviceInfo().GetModelName().c_str(), 
            camera->camera->GetDeviceInfo().GetFullName().c_str(), 
            camera->camera->GetDeviceInfo().GetUserDefinedName().c_str(),
            camera->camera->GetDeviceInfo().GetSerialNumber().c_str()
        );
        if (GenApi::IsAvailable(camera->camera->ResultingFrameRate)) {
            RCLCPP_INFO(this->get_logger(), "Expected resulting frame-rate is %f.", camera->camera->ResultingFrameRate());
        }
        if (GenApi::IsAvailable(camera->camera->ResultingFrameRateAbs)) {
            RCLCPP_INFO(this->get_logger(), "Expected resulting frame-rate (ABS) is %f.", camera->camera->ResultingFrameRateAbs());
        }

        // Camera fully set-up. Now start grabbing.
        camera->camera->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
        grabbing_thread = std::make_unique<std::thread>([this](){
            // have the main loop in a thread since blocking functions and rclcpp::spin() are mutually exclusive
            // see https://answers.ros.org/question/374087/
            while(rclcpp::ok()) {
                try {
                    this->grab_and_publish();
                } catch (const std::runtime_error & e) { // TODO: use more specific exception class
                    RCLCPP_WARN(get_logger(), e.what());
                } catch (const GenICam::TimeoutException & e) {
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
    const std::map<Pylon::EPixelType, const char *> Pylon2ROS {
        {Pylon::EPixelType::PixelType_BayerRG8, sensor_msgs::image_encodings::BAYER_RGGB8},
        {Pylon::EPixelType::PixelType_RGB8packed, sensor_msgs::image_encodings::RGB8},
        {Pylon::EPixelType::PixelType_Mono8, sensor_msgs::image_encodings::MONO8}
    };
    sensor_msgs::msg::Image * pylon_result_to_image_message(Pylon::CGrabResultPtr & ptrGrabResult) {
        sensor_msgs::msg::Image * img = ImageBufferFactory::ReinterpretBufferContext(ptrGrabResult->GetBufferContext());
        img->width = ptrGrabResult->GetWidth();
        img->height = ptrGrabResult->GetHeight();
        size_t stride;
        ptrGrabResult->GetStride(stride);
        img->step = stride;
        const Pylon::EPixelType pixel_type = ptrGrabResult->GetPixelType();
        try {
            img->encoding = Pylon2ROS.at(pixel_type);
        } catch (std::out_of_range &) {
            const std::string pixel_type_str(Pylon::CPixelTypeMapper::GetNameByPixelType(pixel_type));
            RCLCPP_ERROR(this->get_logger(), "Captured image has Pylon pixel type %s. "\
                         "This driver does not know the corresponding ROS image encoding. "\
                         "Please add your preferred pixel type to the mapping code and file a pull request.",
                         pixel_type_str.c_str());
            // I would really like to fail critically on unrecoverable errors, 
            // but throwing exeptions in threads in a composable node
            // messes up the node container
            //throw std::runtime_error("Unknown Pylon pixel type.");
        }
        RCLCPP_DEBUG(this->get_logger(), "Got image %ix%i, stride %zu, size %zu bytes, publishing.", img->width, img->height, stride, ptrGrabResult->GetBufferSize());
        return img; // NOTE: this is NOT a bottleneck. copy-elision is strong in this one.
    }
};

} // end namespace pylon_instant_camera

RCLCPP_COMPONENTS_REGISTER_NODE(pylon_instant_camera::PylonCameraNode)
