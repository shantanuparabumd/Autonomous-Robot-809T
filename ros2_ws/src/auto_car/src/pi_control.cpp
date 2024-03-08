#include <rclcpp/rclcpp.hpp>
#include <pigpio.h>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        // Initialize pigpio library
        if (gpioInitialise() < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio library");
            return;
        }

        // Set pin 16 as an output pin
        gpioSetMode(16, PI_OUTPUT);

        // Create a timer to toggle the pin every second
        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
            // Toggle pin 16
            gpioWrite(16, !gpioRead(16));
        });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
