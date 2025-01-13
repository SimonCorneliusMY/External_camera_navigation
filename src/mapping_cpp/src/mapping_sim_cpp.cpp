#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <iostream>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>



class FPSCounter
{
public:
    FPSCounter() : frame_count(0), start_time(std::chrono::steady_clock::now()) {}

    int fps()
    {
        frame_count++;
        auto elapsed_time = std::chrono::steady_clock::now() - start_time;
        double elapsed_seconds = std::chrono::duration<double>(elapsed_time).count();

        if (elapsed_seconds > 3.0)
        {
            int fps_value = static_cast<int>(frame_count / elapsed_seconds);
            frame_count = 0;
            start_time = std::chrono::steady_clock::now();
            return fps_value;
        }
        return -1; // Return -1 if FPS calculation is not ready
    }

private:
    int frame_count;
    std::chrono::steady_clock::time_point start_time;
};

class Mapping : public rclcpp::Node
{
public:
    Mapping() : Node("mapping"), save_map(false)
    {
        this->declare_parameter("save_map", false);
        this->get_parameter("save_map", save_map);
        rclcpp::QoS qos_profile_reliable(rclcpp::KeepLast(10));
        qos_profile_reliable.transient_local();
        qos_profile_reliable.reliable();

        // qos_profile_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

        bounding_box_sub = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "pose_pixel", qos_profile_reliable, std::bind(&Mapping::bounding_box_callback, this, std::placeholders::_1));

        image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10, std::bind(&Mapping::image_callback, this, std::placeholders::_1));

        map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos_profile_reliable);

        br = std::make_shared<cv_bridge::CvImage>();
        fps_counter = std::make_shared<FPSCounter>();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            // cv::Mat image = br->imgmsg_to_cv2(*msg, "bgr8");
            if (!pose_pixel.empty())
            {
                mapping(cv_ptr->image, pose_pixel, 80);
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
        }
    }

    void bounding_box_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        pose_pixel = msg->data;
    }

    void mapping(const cv::Mat &image, const std::vector<int16_t> &point, int size)
    {
        int x = fps_counter->fps();
            if (x != -1){
                RCLCPP_INFO(this->get_logger(), "FPS: %d", x);
            }
        

        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    

        // Threshold values to isolate floor
        cv::Mat mask_lower, mask_upper, mask;
        cv::inRange(hsv, cv::Scalar(0, 0, 155), cv::Scalar(180, 0, 155), mask_upper);
        cv::inRange(hsv, cv::Scalar(10, 0, 155), cv::Scalar(0, 0, 155), mask_lower);

        mask = mask_lower | mask_upper;

        // Define the square region around the given point
        int half_size = size / 2;
        int start_x = std::max(0, point[1] - half_size);
        int end_x = std::min(mask.rows - 1, point[1] + half_size);
        int start_y = std::max(0, point[0] - half_size);
        int end_y = std::min(mask.cols - 1, point[0] + half_size);

        mask(cv::Range(start_x, end_x + 1), cv::Range(start_y, end_y + 1)) = 255;
        // cv::bitwise_not(mask,mask);

        // Apply morphological opening (erosion followed by dilation)
        cv::Mat kernel = cv::Mat::ones(11, 11, CV_8U);
        cv::Mat mask_open;
        cv::morphologyEx(mask, mask_open, cv::MORPH_OPEN, kernel);

        // Set map values for obstacles and free space
        mask_open = (mask_open == 0) * 100; // Set obstacle = 100, free path = 0

        // Flip the mask (ROS map data has the origin at bottom-left, OpenCV is top-left)
        cv::Mat maze_bw_flip;
        cv::flip(mask_open, maze_bw_flip, 0);

        // Update the map data and publish
        map.header.frame_id = "map";
        map.header.stamp = this->get_clock()->now();
        map.info.height = maze_bw_flip.rows;
        map.info.width = maze_bw_flip.cols;
        map.info.resolution = 4.0 / 1044.0; // 4m = 1044 pixels
        map.info.origin.orientation.w = 1.0;
        map.info.origin.position.x = 0.0;
        map.info.origin.position.y = 0.0;

        //Too lazy to write a proper functions to save images
        if (save_map)
        {
            cv::imwrite("/home/tarumt2204/External_camera_navigation/maze_100.pgm", maze_bw_flip);
            RCLCPP_INFO(this->get_logger(), "Image saved");
            save_map = false;
        }

        // Flatten the map and publish
        map.data.assign(maze_bw_flip.begin<int8_t>(), maze_bw_flip.end<int8_t>());
        map_publisher->publish(map);
    }

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr bounding_box_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;

    nav_msgs::msg::OccupancyGrid map;
    std::shared_ptr<cv_bridge::CvImage> br;
    std::shared_ptr<FPSCounter> fps_counter;
    

    bool save_map;
    std::vector<int16_t> pose_pixel;
    // rclcpp::QoS qos_profile_reliable;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mapping>());
    rclcpp::shutdown();
    return 0;
}
