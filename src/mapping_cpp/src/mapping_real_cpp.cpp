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


#include <algorithm>
#include <numeric>

using std::placeholders::_1;
using std::placeholders::_2;

/*
30/7/26 Maps RGB image to occupancy grid map, uses YOLO bounding box to remove TurtleBot3 from map
void obstacles() is not used in ExPeNav2
*/


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
        this->declare_parameter<double>("min_resolution", 3.45787/1112);
        this->declare_parameter<std::vector<int>>("homographic_ori_points", {636,73,1000, 77,171,531,1278,640});
        this->declare_parameter<std::vector<int>>("homographic_transformed_points", {0,0,1112,0,0,2855,1112,2855});
        this->declare_parameter<std::vector<int>>("HSV", {18, 0, 0, 28, 255, 255});
        this->declare_parameter<int>("inflation", 17);
        this->declare_parameter<int>("morph_size", 2);
        this->declare_parameter<double>("age_penalty", 0.4);

        this->get_parameter("age_penalty", age_penalty);
        this->get_parameter("resolution", map.info.resolution);
        this->get_parameter("save_map", save_map);
        this->get_parameter("name", name);
        this->get_parameter("low_resolution", low_resolution);
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
        sync->setAgePenalty(age_penalty);
        sync->registerCallback(std::bind(&Mapping::SyncCallback, this, _1, _2));
        //pose subscription
        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose", 10, std::bind(&Mapping::pose_callback, this, _1));

        // normal ros2 publisher
        map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_" + name , qos_profile_reliable);
        // obstacle_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle_" + name , qos_profile_reliable);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // wait for transform to determine map bounds
        while (!waitForTransform("map","map_" + name,std::chrono::seconds(10))){
            RCLCPP_INFO(this->get_logger(), "Waiting for transform %s -> %s", "map", ("map_" + name).c_str());
        }

        get_map_bounds();

        br = std::make_shared<cv_bridge::CvImage>();
        

        // Specify the 4 corner coordinates (Z-shape in row-major order, with the origin at the top-left corner)
        pts1 = { cv::Point2f(pts_ori[0], pts_ori[1]), cv::Point2f(pts_ori[2], pts_ori[3]), 
                                        cv::Point2f(pts_ori[4], pts_ori[5]), cv::Point2f(pts_ori[6], pts_ori[7]) };
        pts2 = { cv::Point2f(pts_homo[0], pts_homo[1]), cv::Point2f(pts_homo[2], pts_homo[3]), 
                                        cv::Point2f(pts_homo[4], pts_homo[5]), cv::Point2f(pts_homo[6], pts_homo[7]) };


        M = cv::getPerspectiveTransform(pts1, pts2);

    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose){
        track_pose(pose->pose.position);
    }

    void SyncCallback(const sensor_msgs::msg::Image::ConstSharedPtr & img,
    const my_custom_msgs::msg::Bbox::ConstSharedPtr & bbox){
        // RCLCPP_INFO(this->get_logger(), "Sync callback with %u and %u as times",img->header.stamp.sec, bbox->header.stamp.sec);
        try
        {
            // bounding_box is top left and bottom right coordinates,
            bounding_box = bbox->data;
            
            
            cv::Mat cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8)->image;
            if (cv_ptr.empty()){
                return;
            }

            // RCLCPP_INFO(this->get_logger(), "Condition: %d", (bbox->header.frame_id == "No objects detected" )+ within_map() );
            // not detected, within map 1,0 and 0,1 we map, 1,1 we return, 0,0 we print warning
            switch((bbox->header.frame_id == "No objects detected" )+ within_map()){
                case 1:{
                    mapping(cv_ptr,bounding_box);
                    // print_once = false;
                    break;
                }
                case 2:{
                    return;
                }
                case 0:{
                    // if (!print_once){
                    //     RCLCPP_INFO(this->get_logger(),"YOLO false positive, try increasing confidence threshold in localizer node");
                    //     print_once = true;
                    // }
                    // return;
                    mapping(cv_ptr,bounding_box);
                }
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
        // obstacle_publisher->publish(obstacle);

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


        mask = HSV_thresholding(hsv, hsv_values);

        // For multiple HSV values       
        
        if (hsv_values.size() > 6 && hsv_values.size() % 6 == 0){  
            cv::Mat mask_temp;          
            for (u_int64_t i = 6; i < hsv_values.size(); i+=6){
                mask_temp = HSV_thresholding(hsv, std::vector<int64_t>(hsv_values.begin()+i,hsv_values.begin()+i+6));
                cv::bitwise_or(mask,mask_temp,mask);
            }
        }
        

        TB3_pixel_inflation = this->get_parameter("inflation").get_value<int>();
        // Blackout the TurtleBot3 pose, bounding_box is top left and bottom right coordinates, each box requires 4 values
        if(!bounding_box.empty()){
            
                    
            int x = bounding_box[0]-TB3_pixel_inflation;
            int y = bounding_box[1]-TB3_pixel_inflation;
            int width = bounding_box[2] - bounding_box[0]+TB3_pixel_inflation*2;
            int height = bounding_box[3] - bounding_box[1]+TB3_pixel_inflation*2;

            cv::Rect bbox(x, y, width, height);

            // Black out the area in the image corresponding to the bounding box
            cv::rectangle(mask, bbox, cv::Scalar(255, 255, 255), cv::FILLED);
            
        }
        // Apply the perspective transform
        cv::warpPerspective(mask, homo_transform, M, cv::Size(pts_homo[6], pts_homo[7]));

        // make it a mask again because interpolation in warpPerspective
        cv::threshold(homo_transform,homo_transform,cv::THRESH_BINARY_INV | cv::THRESH_OTSU,100,cv::ThresholdTypes::THRESH_BINARY_INV);

        // Morphology opening (erosion then dilation)
        this->get_parameter("morph_size", morph_size);
        cv::morphologyEx(homo_transform,homo_transform,cv::MORPH_OPEN,element,cv::Point(-1,-1));

       
        map.header.frame_id = "map_" + name;
        map.header.stamp = this->get_clock()->now();
        map.info.height = homo_transform.rows;
        map.info.width = homo_transform.cols;
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

    cv::Mat HSV_thresholding(const cv::Mat &hsv_image, const std::vector<int64_t> &hsv){
        
        try{
            cv::Mat output;
            cv::inRange(hsv_image, cv::Scalar(hsv.at(0), hsv.at(1), hsv.at(2)), cv::Scalar(hsv.at(3), hsv.at(4), hsv.at(5)), output);
            return output;
        }
        catch(const std::out_of_range& e){
            RCLCPP_ERROR(this->get_logger(),"Index out of range");
            return cv::Mat::zeros(hsv_image.rows,hsv_image.cols,CV_64FC1);
        }
    }

    bool waitForTransform(const std::string &target_frame, const std::string &source_frame, std::chrono::seconds timeout)
    {
        auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok())
        {
            try
            {
                // Try to get the transform
                transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                transform.transform.translation.x = transform.transform.translation.x / map.info.resolution;
                transform.transform.translation.y = transform.transform.translation.y / map.info.resolution;
                transform.transform.translation.z = transform.transform.translation.z / map.info.resolution;
                return true; // Transform found
            }
            catch (const tf2::TransformException &e)
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for transform %s -> %s: %s", source_frame.c_str(), target_frame.c_str(), e.what());
            }

            // Check if timeout reached
            auto now = std::chrono::steady_clock::now();
            if (now - start > timeout)
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for transform %s -> %s", source_frame.c_str(), target_frame.c_str());
                return false; // Timeout reached
            }

            // Sleep briefly before retrying
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }

        return false; // In case of an unexpected shutdown
    }  
    bool within_map(){
        try{
            
            if (poses.empty()){
                return false;
            }
            // RCLCPP_INFO(this->get_logger(), "Pose x %f y %f , Map bound min x %f y %f, max x %f max y %f",
            //     poses.back().x, poses.back().y, map_bounds.at("min").at(0),map_bounds.at("min").at(1),map_bounds.at("max").at(0),map_bounds.at("max").at(1));
            // Check if x is within bounds
            if (poses.back().x < map_bounds.at("min").at(0) - map_transition_length || poses.back().x > map_bounds.at("max").at(0) + map_transition_length) {
;
                return false;
            }
            
            // Check if y is within bounds
            if (poses.back().y < map_bounds.at("min").at(1) - map_transition_length || poses.back().y > map_bounds.at("max").at(1) + map_transition_length) {
                return false;
            }
            
            // If both x and y are within bounds, return true
            return true;
        }
        catch(const std::out_of_range& e){
            RCLCPP_ERROR(this->get_logger(), "Error in within map %s", e.what());
            return false;
        }
    }

    void get_map_bounds() {

        double map_inflation_metres = 0.8;
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();
        std::vector<geometry_msgs::msg::Point> points(4);

        int h = pts_homo[7];
        int w = pts_homo[6];
    
        points[0].set__x(0.0); points[0].set__y(0.0); points[0].set__z(0.0);  // Bottom-left
        points[1].set__x(0.0); points[1].set__y(h); points[1].set__z(0.0);    // Top-left
        points[2].set__x(w); points[2].set__y(0.0); points[2].set__z(0.0);    // Bottom-right
        points[3].set__x(w); points[3].set__y(h); points[3].set__z(0.0);      // Top-right
        // Transform all four corners and find the bounding box
        for (auto& pt : points) {
            geometry_msgs::msg::Point transformed_pt;
            // Apply static transform
            tf2::doTransform(pt, transformed_pt, transform);
            // Keep track of min and max points
            min_x = std::min(min_x, transformed_pt.x);
            min_y = std::min(min_y, transformed_pt.y);
            max_x = std::max(max_x, transformed_pt.x);
            max_y = std::max(max_y, transformed_pt.y);
        }
        
            

        // store the map bounds in meters
        map_bounds["min"].push_back(min_x * map.info.resolution - map_inflation_metres);
        map_bounds["min"].push_back(min_y * map.info.resolution - map_inflation_metres);
        map_bounds["max"].push_back(max_x * map.info.resolution + map_inflation_metres);
        map_bounds["max"].push_back(max_y * map.info.resolution + map_inflation_metres);
        RCLCPP_INFO(this->get_logger(), "Map bounds: min (%f,%f), max (%f,%f)",
            map_bounds.at("min").at(0), map_bounds.at("min").at(1),
            map_bounds.at("max").at(0), map_bounds.at("max").at(1));
        

    }
    void erase_tb3(const cv::Mat &image, std::vector<int16_t> &bounding_box)
    {
        // Blackout the TurtleBot3 pose, bounding_box is top left and bottom right coordinates, each box requires 4 values
        if(bounding_box.empty()){
            RCLCPP_ERROR(this->get_logger(),"Bounding box empty");

        }else{
            int x = bounding_box[0]-TB3_pixel_inflation;
            int y = bounding_box[1]-TB3_pixel_inflation;
            int width = bounding_box[2] - bounding_box[0]+TB3_pixel_inflation*2;
            int height = bounding_box[3] - bounding_box[1]+TB3_pixel_inflation*2;

            cv::Rect bbox(x, y, width, height);

            // Black out the area in the image corresponding to the bounding box
            cv::rectangle(image, bbox, cv::Scalar(255, 255, 255), cv::FILLED);
        }
    }
    void track_pose(const geometry_msgs::msg::Point & pose){

        poses.push(pose);
        if (poses.size() > 3){
            poses.pop();
        }
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped transform;
    
    
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
    bool save_map,bbox_image_check,print_once = false;
    std::vector<int16_t> bounding_box;
    std::unordered_map<std::string,std::vector<double>> map_bounds;
    std::queue<geometry_msgs::msg::Point> poses;
    int morph_size =2;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );


    cv::Point2f pose_xy_homo;
    cv::Point2f pose_xy_pixels_homo;
    double low_resolution, map_transition_length = 0.1,age_penalty;

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
