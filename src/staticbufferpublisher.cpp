#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <chrono> // for 1s literal
using namespace std::chrono_literals;

template <typename T>
class FixedBufferAllocator: public std::allocator<T>
{
public:
    typedef size_t size_type;
    typedef T* pointer;
    typedef const T* const_pointer;
    typedef const T& const_reference;
    template<typename _Tp1>
    struct rebind { typedef FixedBufferAllocator<_Tp1> other; };
    pointer allocate(size_type n, const void * hint = 0) {
        std::cerr << "allocate" << std::endl;
        return std::allocator<T>::allocate(n, hint);
    }
    void deallocate(pointer p, size_type n) {
        std::cerr << "deallocate" << std::endl;
        return std::allocator<T>::deallocate(p, n);
    }
    void construct(pointer p, const_reference val) {
        std::cerr << "construct" << std::endl;
        return std::allocator<T>::construct(p, val);
    }
    FixedBufferAllocator() throw(): std::allocator<T>() {
        std::cerr << "FixedBufferAllocator" << std::endl;
    }
    FixedBufferAllocator(const FixedBufferAllocator &a) throw(): std::allocator<T>(a) { }
    template <class U> FixedBufferAllocator(const FixedBufferAllocator<U> &a) throw(): std::allocator<T>(a) { }
    ~FixedBufferAllocator() throw() { }
};
using Alloc = FixedBufferAllocator<sensor_msgs::msg::Image>;

/*
class FixedBufferAllocator
{
private:
std::allocator<sensor_msgs::msg::Image> alloc;
public:
    typedef size_t size_type;
    typedef sensor_msgs::msg::Image* pointer;
    typedef const sensor_msgs::msg::Image* const_pointer;
    typedef const sensor_msgs::msg::Image& const_reference;
    struct rebind { typedef FixedBufferAllocator other; };
    pointer allocate(size_type n, const void * hint = 0) {
        std::cerr << "allocate" << std::endl;
        return alloc.allocate(n, hint);
    }
    void deallocate(pointer p, size_type n) {
        std::cerr << "deallocate" << std::endl;
        return alloc.deallocate(p, n);
    }
    void construct(pointer p, const_reference val) {
        std::cerr << "construct" << std::endl;
        return alloc.construct(p, val);
    }
    FixedBufferAllocator() throw() : alloc() {
        std::cerr << "FixedBufferAllocator" << std::endl;
    }
};
using Alloc = FixedBufferAllocator;
*/
using MessageAllocTraits =
rclcpp::allocator::AllocRebind<sensor_msgs::msg::Image, Alloc>;
using MessageAlloc = MessageAllocTraits::allocator_type;
using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, sensor_msgs::msg::Image>;
using MessageUniquePtr = std::unique_ptr<sensor_msgs::msg::Image, MessageDeleter>;
  
class StaticBufferPublisher : public rclcpp::Node {
private:
    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher;
    rclcpp::TimerBase::SharedPtr timer;
    sensor_msgs::msg::Image buffer;
public:
    StaticBufferPublisher(const rclcpp::NodeOptions & options) : rclcpp::Node("StaticBufferPublisher", options) {
        if (!options.use_intra_process_comms()) {
            throw std::runtime_error("Must use use_intra_process_comms!");
        }
        auto alloc = std::make_shared<Alloc>();
        rclcpp::PublisherOptionsWithAllocator<Alloc> publisher_options;
        publisher_options.allocator = alloc;
        auto imagePublisher = this->create_publisher<sensor_msgs::msg::Image>("image", 1, publisher_options);
        buffer.data.resize(1024); // prepare buffer once
        // camera.set_buffer(buffer.data.data()) // tell camera where to put the data
        // camera.start_grabbing() // start camera
        // this timer simulates a while(camera.block_until_image_ready())
        timer = this->create_wall_timer(1s, [this, imagePublisher]() -> void {
            std::cerr << "Sending  data starting at " << static_cast<void*>(buffer.data.data()) << "." << std::endl;
            imagePublisher->publish(buffer); // this works, but creates a copy
            //std::unique_ptr<sensor_msgs::msg::Image> img_msg_ptr(&buffer);
            //imagePublisher->publish(std::move(img_msg_ptr)); // this does not create a copy, but releases the buffer
        });
    }
};
RCLCPP_COMPONENTS_REGISTER_NODE(StaticBufferPublisher)
