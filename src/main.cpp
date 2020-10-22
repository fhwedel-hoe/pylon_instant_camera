#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <camera_info_manager/camera_info_manager.h>

class PylonUSBCamera {
    private:
        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;
        Pylon::CBaslerUsbInstantCamera * camera;
    public:
        PylonUSBCamera() {
            Pylon::PylonInitialize();
            // Create an instant camera object with the camera device found first.
            camera = new Pylon::CBaslerUsbInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
            camera->Open();
            // Print the model name of the camera.
            ROS_INFO("Using device %s", camera->GetDeviceInfo().GetModelName().c_str());
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
            camera->StartGrabbing(/*Pylon::GrabStrategy_LatestImageOnly*/);
        }
        ~PylonUSBCamera() {
            delete camera;
            Pylon::PylonTerminate(); 
        }
    sensor_msgs::Image grab_frame() {
        if (camera->IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 1000 ms is used.
            camera->RetrieveResult(1000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
                const size_t payloadSize = ptrGrabResult->GetPayloadSize();

                // TODO: return ptrGrabResult, create sensor_msgs::Image envelope outside this function 
                sensor_msgs::Image img_raw_msg_;
                img_raw_msg_.header.stamp = ros::Time::now(); 
                img_raw_msg_.width = ptrGrabResult->GetWidth();
                img_raw_msg_.height = ptrGrabResult->GetHeight();
                img_raw_msg_.encoding = sensor_msgs::image_encodings::RGB8; // TODO: check if this actually matches the camera settings
                img_raw_msg_.step = img_raw_msg_.width * 3; // TODO: see above
                img_raw_msg_.data.assign(pImageBuffer,pImageBuffer+payloadSize);
                return img_raw_msg_;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pylon_camera_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher publisher = it.advertiseCamera("image_raw", 1);
    camera_info_manager::CameraInfoManager cim(nh);
    std::string camera_info_url;
    if (nh.getParam("camera_info_url", camera_info_url))
    {
        cim.loadCameraInfo(camera_info_url);
    }
    else
    {
        ROS_WARN("camera_info_url not supplied in configuration. Camera info will be unavailable. Rectification is not possible.");
    }
    sensor_msgs::CameraInfo cam_info = cim.getCameraInfo();
    PylonUSBCamera camera;
    while(ros::ok()) {
        sensor_msgs::Image img = camera.grab_frame();
        cam_info.header.stamp = img.header.stamp;
        publisher.publish(img, cam_info);
        ros::spinOnce();
    };
    return EXIT_SUCCESS;
}
