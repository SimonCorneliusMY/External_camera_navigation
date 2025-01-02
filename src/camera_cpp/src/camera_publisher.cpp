#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <chrono>
#include <string>
#include <iostream>

class WebcamPublisher : public rclcpp::Node {
public:
    WebcamPublisher()
        : Node("webcam_publisher"), cap_(0, cv::CAP_V4L2) {

        rclcpp::QoS profile(rclcpp::KeepLast(10));
        profile.best_effort();
        // Create the image publisher and FPS publisher
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", profile);
        fps_publisher_ = this->create_publisher<std_msgs::msg::Int32>("fps", 10);

        // Set up a timer to run the callback at 10Hz (every 100 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&WebcamPublisher::timer_callback, this));
        

        // Open the default webcam
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));  // MJPEG format
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);  // Set width to 1280px
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720); // Set height to 720px

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open webcam.");
            rclcpp::shutdown();
            return;
        }

        // Capture the initial time
        last_time_ = std::chrono::steady_clock::now();
    }

private:
    void timer_callback() {
        // Capture a frame from the webcam

        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture image.");
            return;
        }

        // Convert the OpenCV image to ROS Image message
        auto ros_image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        auto image_ptr = std::make_shared<sensor_msgs::msg::Image>(*ros_image);
  
        // Publish the image
        image_publisher_->publish(*image_ptr);

        // Log the first publish
        if (first_publish_) {
            RCLCPP_INFO(this->get_logger(), "Publishing webcam feed...");
            first_publish_ = false;
        }

        // Calculate FPS based on the time elapsed since the last frame
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_time_;
        double fps = 1.0 / elapsed.count();


             
        // Publish FPS
        std_msgs::msg::Int32 fps_msg;
        fps_msg.data = static_cast<int32_t>(fps);
        
        fps_publisher_->publish(fps_msg);

        // Update last_time_ to the current time
        last_time_ = now;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fps_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;  // OpenCV VideoCapture object
    bool first_publish_ = true;
    cv::Mat frame;
    std::chrono::steady_clock::time_point last_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebcamPublisher>());
    rclcpp::shutdown();
    return 0;
}
