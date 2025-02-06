#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/int16_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <iostream>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "my_custom_msgs/msg/bbox.hpp"


#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <vector>
#include <algorithm>
#include <numeric>

using std::placeholders::_1;
using std::placeholders::_2;

class FPSCounter
{
public:
    struct FPSStats
    {
        int fps_value;
        double min_time;
        double max_time;
        double avg_time;
    };

    FPSCounter() : frame_count(0), start_time(std::chrono::steady_clock::now()) {}

    FPSStats fps()
    {
        frame_count++;
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = current_time - start_time;
        double elapsed_seconds = std::chrono::duration<double>(elapsed_time).count();

        // Record the time taken for each frame
        frame_times.push_back(elapsed_seconds);

        FPSStats stats = {-1, 0.0, 0.0, 0.0};

        if (elapsed_seconds > 3.0)
        {
            stats.fps_value = static_cast<int>(frame_count / elapsed_seconds);
            frame_count = 0;
            start_time = current_time;

            // Calculate the minimum, maximum, and average time taken per frame
            stats.min_time = *std::min_element(frame_times.begin(), frame_times.end());
            stats.max_time = *std::max_element(frame_times.begin(), frame_times.end());
            stats.avg_time = std::accumulate(frame_times.begin(), frame_times.end(), 0.0) / frame_times.size();

            // Clear the frame time history for the next calculation window
            frame_times.clear();
        }

        return stats; // Return the collected statistics
    }

private:
    int frame_count;
    std::chrono::steady_clock::time_point start_time;
    std::vector<double> frame_times; // Store the frame times
};



class Mapping : public rclcpp::Node
{
public:
    Mapping() : Node("mapping")
    {
        this->declare_parameter("save_map", false);
        this->declare_parameter<bool>("show_fps", false);
        this->get_parameter("save_map", save_map);
        rclcpp::QoS qos_profile_reliable(rclcpp::KeepLast(1));
        qos_profile_reliable.transient_local();
        qos_profile_reliable.reliable();
        cv::namedWindow("image",cv::WINDOW_NORMAL);
        cv::namedWindow("image2",cv::WINDOW_NORMAL);
        cv::resizeWindow("image",600,1200);
        cv::resizeWindow("image2",600,1200);




        // qos_profile_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

        // bounding_box_sub = this->create_subscription<my_custom_msgs::msg::Bbox>(
        //     "bounding_box", 10, std::bind(&Mapping::bounding_box_callback, this, std::placeholders::_1));

        // image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        //     "camera/image_raw", 10, std::bind(&Mapping::image_callback, this, std::placeholders::_1));
        image_sub.subscribe(this,"camera/image_raw");
        bounding_box_sub.subscribe(this,"bounding_box");

        uint32_t queue_size = 10;

        sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::
            ApproximateTime<sensor_msgs::msg::Image, my_custom_msgs::msg::Bbox>>>(
                message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                my_custom_msgs::msg::Bbox>(queue_size), image_sub,bounding_box_sub);


        sync->setAgePenalty(0.5);
        sync->registerCallback(std::bind(&Mapping::SyncCallback, this, _1, _2));

        map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos_profile_reliable);
        obstacle_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle", qos_profile_reliable);

        br = std::make_shared<cv_bridge::CvImage>();
        

        // Specify the 4 corner coordinates (Z-shape in row-major order)
        pts1 = { cv::Point2f(599, 98), cv::Point2f(938, 104), 
                                        cv::Point2f(72, 532), cv::Point2f(1275, 637) };
        pts2 = { cv::Point2f(0, 0), cv::Point2f(1203, 0), 
                                        cv::Point2f(0, 2869), cv::Point2f(1203, 2869) };

        // Calculate perspective transform matrix (done once during object creation)
        M = cv::getPerspectiveTransform(pts1, pts2);

    }

private:

    void SyncCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img,
    const my_custom_msgs::msg::Bbox::ConstSharedPtr & bbox){
        // RCLCPP_INFO(this->get_logger(), "Sync callback with %u and %u as times",img->header.stamp.sec, bbox->header.stamp.sec);
        try
        {
            bounding_box = bbox->data;

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            // cv::Mat image = br->imgmsg_to_cv2(*msg, "bgr8");
            if (!bounding_box.empty() && !cv_ptr->image.empty())
            {

                mapping(cv_ptr->image, bounding_box);
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
        }
        
    }
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            // cv::Mat image = br->imgmsg_to_cv2(*msg, "bgr8");
            if (!bounding_box.empty() && !cv_ptr->image.empty())
            {
                mapping(cv_ptr->image, bounding_box);
                bbox_image_check = true;

            }
            else if (bbox_image_check == true)
            {
                RCLCPP_INFO(this->get_logger(), "No image or bounding box");
                bbox_image_check = false;
                return;
            }

            
            
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
        }
    }

    void bounding_box_callback(const my_custom_msgs::msg::Bbox::SharedPtr msg)
    {
        bounding_box = msg->data;

        // Printing the entire array (elements) 
        // RCLCPP_INFO(this->get_logger(), "Received data:");
        //     for (size_t i = 0; i < msg->data.size(); ++i)
        //     {
        //         RCLCPP_INFO(this->get_logger(), "Element %zu: %d", i, msg->data[i]);
        //     }

    }
    // TODO the thresholding has some weird issue where causing the mask to have multiple value
    void mapping(const cv::Mat &image,  std::vector<int16_t> &bounding_box)
    {
        // to show fps of node
        bool show_fps = this -> get_parameter("show_fps").get_value<bool>();
        if (show_fps == true){
            FPSCounter::FPSStats stats = counter.fps();
            if (stats.fps_value != -1)
            {
                RCLCPP_INFO(this->get_logger(), "FPS: %d, Min: %f, Max: %f, Avg: %f",stats.fps_value,stats.min_time,stats.max_time,stats.avg_time);
            }
        }

        // Threshold values to isolate floor
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(10, 0, 0), cv::Scalar(30, 255, 255), mask);

        // Blackout the TurtleBot3 pose, bounding_box is top left and bottom right coordinates, each box requires 4 values
        for (size_t i = 0; i < bounding_box.size(); i += 4)
            {
                int x = bounding_box[i]-50;
                int y = bounding_box[i + 1]-50;
                int width = bounding_box[i + 2] - bounding_box[i]+100;
                int height = bounding_box[i + 3] - bounding_box[i + 1]+100;

                // Create a cv::Rect for each bounding box
                cv::Rect bbox(x, y, width, height);

                // Black out the area in the image corresponding to the bounding box
                cv::rectangle(mask, bbox, cv::Scalar(255, 255, 255), cv::FILLED);
        }

        // Apply the perspective transform
        cv::warpPerspective(mask, homo_transform, M, cv::Size(1203, 2869));
        // make it a mask again because interpolation in warpPerspective
        cv::threshold(homo_transform,homo_transform,cv::THRESH_BINARY_INV | cv::THRESH_OTSU,100,cv::ThresholdTypes::THRESH_BINARY_INV);


        // cv::imshow("image2",homo_transform);
        // cv::waitKey(1);

        // Set map values for obstacles and free space
        // homo_transform.setTo(0,homo_transform==255); // Set obstacle = 100, free path = 0
        // mask.setTo(0,mask==255);

        // Create a set to store unique values
        // std::set<int> uniqueValues;

        // // Iterate through each element in the matrix and add it to the set
        // for (int i = 0; i < mask.rows; ++i) {
        //     for (int j = 0; j < mask.cols; ++j) {
        //         uniqueValues.insert(mask.at<uchar>(i, j));
        //     }
        // }
        // for (int val : uniqueValues){
        //     RCLCPP_INFO(this->get_logger(), "%d",val);
        // }

        cv::imshow("image2",homo_transform);
        cv::imshow("image",mask);
        cv::waitKey(1);
        // Flip the mask (ROS map data has the origin at bottom-left, OpenCV is top-left)
        cv::flip(homo_transform, maze_bw_flip, 0);

        // Update the map data and publish

        map.header.frame_id = "map";
        map.header.stamp = this->get_clock()->now();
        map.info.height = maze_bw_flip.rows;
        map.info.width = maze_bw_flip.cols;
        map.info.resolution = 3.54/1203; // 3.54/1203
        map.info.origin.orientation.w = 1.0;
        map.info.origin.position.x = 0.0;
        map.info.origin.position.y = 0.0;

        //Too lazy to write a proper functions to save images
        if (save_map)
        {
            cv::imwrite("/home/tarumt2204/External_camera_navigation/maze_100.pgm", homo_transform);
            RCLCPP_INFO(this->get_logger(), "Image saved");
            save_map = false;
        }

        // Flatten the map and publish
        map.data.assign(maze_bw_flip.begin<int8_t>(), maze_bw_flip.end<int8_t>());
        map_publisher->publish(map);

        // RCLCPP_INFO(this->get_logger(), "%d,%d",homo_transform.channels(),maze_bw_flip.channels());
        // Get an iterator to the end of the unique values
        // auto unique_end = std::unique(homo_transform.begin<int8_t>(), homo_transform.end<int8_t>());
        


        // // Iterate through the unique values and print them
        // RCLCPP_INFO(this->get_logger(), "Unique values:");

        // for (auto it = homo_transform.begin<int8_t>(); it != unique_end; ++it) {
        //     RCLCPP_INFO(this->get_logger(), "%d", *it);
        // }



                // Create the PointCloud2 message
        sensor_msgs::msg::PointCloud2 pointcloud;
        
        // Set the header of the point cloud
        pointcloud.header.stamp = rclcpp::Clock().now();
        pointcloud.header.frame_id = "map"; // Set the frame_id for the point cloud
        
        // Set height and width of the point cloud
        pointcloud.height = 1;  // Single row of points (unordered)
        
        // Point data (coordinates in 3D space)
        std::vector<geometry_msgs::msg::Point32> points;
        
        // Iterate through the occupancy grid and add occupied cells (value 100) as points
        const auto& origin = map.info.origin;
        const float resolution = map.info.resolution;
        
        for (size_t y = 0; y < map.info.height; ++y)
        {
            for (size_t x = 0; x < map.info.width; ++x)
            {
                // Occupied cell value is 100
                if (map.data[y * map.info.width + x] == 100)
                {

                    // Convert grid coordinates (x, y) to world coordinates (X, Y)
                    geometry_msgs::msg::Point32 pt;
                    pt.x = origin.position.x + x * resolution + resolution / 2.0;
                    pt.y = origin.position.y + y * resolution + resolution / 2.0;
                    pt.z = 0.0;  // Z is always 0 in 2D grid

                    points.push_back(pt);
                }
            }
        }
        pointcloud.width = points.size();  // One point for each cell in the grid
        // Create PointFields for the point cloud
        std::vector<sensor_msgs::msg::PointField> fields;

        // Add X, Y, Z field (for 3D point cloud data)
        sensor_msgs::msg::PointField x_field;
        x_field.name = "x";
        x_field.offset = 0;
        x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        x_field.count = 1;
        fields.push_back(x_field);

        sensor_msgs::msg::PointField y_field;
        y_field.name = "y";
        y_field.offset = 4;  // Offset in bytes (float is 4 bytes)
        y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        y_field.count = 1;
        fields.push_back(y_field);

        sensor_msgs::msg::PointField z_field;
        z_field.name = "z";
        z_field.offset = 8;
        z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        z_field.count = 1;
        fields.push_back(z_field);

        // Assign fields to pointcloud message
        pointcloud.fields = fields;
        
        // Point step (size of one point in bytes)
        pointcloud.point_step = 12;  // 3 floats (x, y, z), each float is 4 bytes
        pointcloud.row_step = pointcloud.point_step * points.size();
        pointcloud.is_bigendian = false; // Little endian
        pointcloud.is_dense = true; // All points are valid (no NaNs or invalid points)
        
        // Create a buffer for the point data
        pointcloud.data.resize(pointcloud.row_step);

        // Fill the data array with point information
        size_t i = 0;
        for (const auto& pt : points)
        {
        memcpy(&pointcloud.data[i], &pt.x, sizeof(pt.x));  // Copy x
        i += sizeof(pt.x);
        memcpy(&pointcloud.data[i], &pt.y, sizeof(pt.y));  // Copy y
        i += sizeof(pt.y);
        memcpy(&pointcloud.data[i], &pt.z, sizeof(pt.z));  // Copy z
        i += sizeof(pt.z);
        }

        // Publish the PointCloud2 message
        obstacle_publisher->publish(pointcloud);

    }




    // rclcpp::Subscription<my_custom_msgs::msg::Bbox>::SharedPtr bounding_box_sub;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher;

    sensor_msgs::msg::PointCloud2 obstacle;
    nav_msgs::msg::OccupancyGrid map;
    std::shared_ptr<cv_bridge::CvImage> br;
    std::shared_ptr<FPSCounter> fps_counter;
    

    bool save_map,bbox_image_check;
    std::vector<int16_t> bounding_box;
    cv::Point2f pose_xy_homo;
    cv::Point2f pose_xy_pixels_homo;
    bool show_homographic_region = true;
    float homo_resolution = 0.1;
    cv::Mat homo_transform, M, hsv, mask, mask_open, maze_bw_flip;
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    FPSCounter counter;

    message_filters::Subscriber<my_custom_msgs::msg::Bbox> bounding_box_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub;
    
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, my_custom_msgs::msg::Bbox>>> sync;


    // my_custom_msgs::msg::Bbox bounding_box;
    

    
    // rclcpp::QoS qos_profile_reliable;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mapping>());
    rclcpp::shutdown();
    return 0;
}
