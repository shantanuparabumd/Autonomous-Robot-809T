#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"  // Included for cv_bridge (OPENCV TO ROS)
#include <opencv2/highgui.hpp>  // Included for highgui  (OPENCV)


class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        // Create publisher for image topic
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", 10);

        // Set up camera capture
        capture_ = cv::VideoCapture(0);
        if (!capture_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
            return;
        }

        // Start publishing camera frames
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraNode::publishImage, this));
    }

private:
    void publishImage()
    {
        cv::Mat frame;
        capture_ >> frame;

        if (!frame.empty())
        {
            // Convert OpenCV image to ROS image message
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

            // Publish the image message
            image_publisher_->publish(*msg);
        }
    }

    cv::VideoCapture capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
