#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
class AddressPrinter : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
public:
    AddressPrinter(const rclcpp::NodeOptions & options) : rclcpp::Node("AddressPrinter", options) {
        if (!options.use_intra_process_comms()) {
            throw std::runtime_error("Must use use_intra_process_comms!");
        }
        imageSubscription = this->create_subscription<sensor_msgs::msg::Image>("image", 1,
            [this](sensor_msgs::msg::Image::UniquePtr msg) {
                std::cerr << "Received data starting at " << static_cast<void*>(msg->data.data()) << "." << std::endl;
            }
        );
    }
};
RCLCPP_COMPONENTS_REGISTER_NODE(AddressPrinter)
