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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>


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
        this->declare_parameter<std::string>("name", "0");
        this->declare_parameter<double>("resolution", 3.45787/1112);
        this->declare_parameter<std::vector<int>>("homographic_ori_points", {636,73,1000, 77,171,531,1278,640});
        this->declare_parameter<std::vector<int>>("homographic_transformed_points", {0,0,1112,0,0,2855,1112,2855});
        this->declare_parameter<std::vector<int>>("HSV", {18, 0, 0, 28, 255, 255});
        this->declare_parameter<int>("inflation", 17);
        
        this->get_parameter("resolution", map.info.resolution);
        this->get_parameter("save_map", save_map);
        this->get_parameter("name", name);
        pts_ori = this->get_parameter("homographic_ori_points").as_integer_array();
        pts_homo = this->get_parameter("homographic_transformed_points").as_integer_array();
        rclcpp::QoS qos_profile_reliable(rclcpp::KeepLast(1));
        qos_profile_reliable.transient_local();
        qos_profile_reliable.reliable();
        // cv::namedWindow("image",cv::WINDOW_NORMAL);
        // cv::namedWindow("image2",cv::WINDOW_NORMAL);
        // cv::resizeWindow("image",600,1200);
        // cv::resizeWindow("image2",600,1200);

        // subscription topic using the sync package
        image_sub.subscribe(this,"camera_" + name +  "/image_raw");
        bounding_box_sub.subscribe(this,"bounding_box_" + name);

        uint32_t queue_size = 10;

        sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::
            ApproximateTime<sensor_msgs::msg::Image, my_custom_msgs::msg::Bbox>>>(
                message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                my_custom_msgs::msg::Bbox>(queue_size), image_sub,bounding_box_sub);

        // limit of sync mismatch
        sync->setAgePenalty(0.5);
        sync->registerCallback(std::bind(&Mapping::SyncCallback, this, _1, _2));

        // normal ros2 publisher
        map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_" + name , qos_profile_reliable);
        obstacle_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle_" + name , qos_profile_reliable);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        br = std::make_shared<cv_bridge::CvImage>();
        
        
        // Specify the 4 corner coordinates (Z-shape in row-major order)
        pts1 = { cv::Point2f(pts_ori[0], pts_ori[1]), cv::Point2f(pts_ori[2], pts_ori[3]), 
                                        cv::Point2f(pts_ori[4], pts_ori[5]), cv::Point2f(pts_ori[6], pts_ori[7]) };
        pts2 = { cv::Point2f(pts_homo[0], pts_homo[1]), cv::Point2f(pts_homo[2], pts_homo[3]), 
                                        cv::Point2f(pts_homo[4], pts_homo[5]), cv::Point2f(pts_homo[6], pts_homo[7]) };


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
                obstacles();
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
        }
        
    }
    void obstacles(){
        // clear points to avoid build up of data in it.
        points.clear();
        
        // Set the header of the point cloud
        obstacle.header.stamp = rclcpp::Clock().now();
        obstacle.header.frame_id = "map"; // Set the frame_id for the point cloud
        
        // Set height and width of the point cloud
        obstacle.height = 1;  // Single row of points (unordered)
        

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

                    pt.x = origin.position.x + x * resolution + resolution / 2.0;
                    pt.y = origin.position.y + y * resolution + resolution / 2.0;
                    pt.z = 0.0;  // Z is always 0 in 2D grid

                    points.push_back(pt);
                }
            }
        }
        obstacle.width = points.size();  // One point for each cell in the grid
        // Create PointFields for the point cloud
        

        // Add X, Y, Z field (for 3D point cloud data)

        x_field.name = "x";
        x_field.offset = 0;
        x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        x_field.count = 1;
        fields.push_back(x_field);

        y_field.name = "y";
        y_field.offset = 4;  // Offset in bytes (float is 4 bytes)
        y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        y_field.count = 1;
        fields.push_back(y_field);

        z_field.name = "z";
        z_field.offset = 8;
        z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        z_field.count = 1;
        fields.push_back(z_field);

        // Assign fields to pointcloud message
        obstacle.fields = fields;
        
        // Point step (size of one point in bytes)
        obstacle.point_step = 12;  // 3 floats (x, y, z), each float is 4 bytes
        obstacle.row_step = obstacle.point_step * points.size();
        obstacle.is_bigendian = false; // Little endian
        obstacle.is_dense = true; // All points are valid (no NaNs or invalid points)

        

        // Create a buffer for the point data
        obstacle.data.resize(obstacle.row_step);


        size_t i = 0;
        for (const auto& pt : points)
        {
            memcpy(&obstacle.data[i], &pt.x, sizeof(pt.x));  // Copy x
            i += sizeof(pt.x);
            memcpy(&obstacle.data[i], &pt.y, sizeof(pt.y));  // Copy y
            i += sizeof(pt.y);
            memcpy(&obstacle.data[i], &pt.z, sizeof(pt.z));  // Copy z
            i += sizeof(pt.z);
        }

        // Publish the PointCloud2 message
        obstacle_publisher->publish(obstacle);

    }
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
        hsv_values = this->get_parameter("HSV").as_integer_array();
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(hsv_values[0], hsv_values[1], hsv_values[2]), cv::Scalar(hsv_values[3], hsv_values[4], hsv_values[5]), mask);

        TB3_pixel_inflation = this->get_parameter("inflation").get_value<int>();
        // Blackout the TurtleBot3 pose, bounding_box is top left and bottom right coordinates, each box requires 4 values
        for (size_t i = 0; i < bounding_box.size(); i += 4)
            {
                
                int x = bounding_box[i]-TB3_pixel_inflation;
                int y = bounding_box[i + 1]-TB3_pixel_inflation;
                int width = bounding_box[i + 2] - bounding_box[i]+TB3_pixel_inflation*2;
                int height = bounding_box[i + 3] - bounding_box[i + 1]+TB3_pixel_inflation*2;

                // Create a cv::Rect for each bounding box
                cv::Rect bbox(x, y, width, height);

                // Black out the area in the image corresponding to the bounding box
                cv::rectangle(mask, bbox, cv::Scalar(255, 255, 255), cv::FILLED);
        }
        
        // Apply the perspective transform
        cv::warpPerspective(mask, homo_transform, M, cv::Size(pts_homo[6], pts_homo[7]));
        // make it a mask again because interpolation in warpPerspective
        cv::threshold(homo_transform,homo_transform,cv::THRESH_BINARY_INV | cv::THRESH_OTSU,100,cv::ThresholdTypes::THRESH_BINARY_INV);

        // Create a set to store unique values, keep for checking unique value 250206 Simon
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

        // cv::imshow("image2",homo_transform);
        // cv::imshow("image",mask);
        // cv::waitKey(1);
        // Flip the mask (ROS map data has the origin at bottom-left, OpenCV is top-left)
        // cv::flip(homo_transform, maze_bw_flip, 0);
       
        map.header.frame_id = "map_" + name;
        map.header.stamp = this->get_clock()->now();
        map.info.height = homo_transform.rows;
        map.info.width = homo_transform.cols;
        // map.info.resolution = 3.54/1203; // 3.54/1203
        // map.info.origin.orientation.x = transform.transform.rotation.x;;
        // map.info.origin.orientation.y = transform.transform.rotation.y;;
        // map.info.origin.orientation.z = transform.transform.rotation.z;;
        map.info.origin.orientation.w = 1.0;;
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
        map.data.assign(homo_transform.begin<int8_t>(), homo_transform.end<int8_t>());
        map_publisher->publish(map);

    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    
    sensor_msgs::msg::PointCloud2 obstacle;
    std::vector<geometry_msgs::msg::Point32> points;
    nav_msgs::msg::OccupancyGrid map;
    std::shared_ptr<cv_bridge::CvImage> br;
    std::shared_ptr<FPSCounter> fps_counter;

    // PointCloud2 variables
    sensor_msgs::msg::PointField x_field, y_field, z_field;
    std::vector<sensor_msgs::msg::PointField> fields;
    geometry_msgs::msg::Point32 pt;
    
    std::string name;
    bool save_map,bbox_image_check;
    std::vector<int16_t> bounding_box;

    cv::Point2f pose_xy_homo;
    cv::Point2f pose_xy_pixels_homo;
    

    std::chrono::steady_clock::time_point t1, t2, t3 ;
    cv::Mat homo_transform, M, hsv, mask, mask_open, maze_bw_flip;
    std::vector<cv::Point2f> pts1,pts2;
    FPSCounter counter;
    int TB3_pixel_inflation, size = 0;   //erases the surrounding pixels
    std::vector<int64_t> pts_ori, pts_homo, hsv_values;

    message_filters::Subscriber<my_custom_msgs::msg::Bbox> bounding_box_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub;
    
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, my_custom_msgs::msg::Bbox>>> sync;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mapping>());
    rclcpp::shutdown();
    return 0;
}
