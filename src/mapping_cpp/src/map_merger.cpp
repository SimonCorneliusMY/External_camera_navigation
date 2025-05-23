#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
class MapMergerNode : public rclcpp::Node
{
public:
    MapMergerNode() : Node("map_merger_node")
    {
        // Initialize tf2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        this->declare_parameter<double>("resolution", 3.45787/980);
        this->declare_parameter<std::vector<std::string>>("camera_addresses", {"0","1"});
        this->declare_parameter("local_costmap.inflation_layer.inflation_radius", 0.55);
        this->get_parameter("camera_addresses",camera_addresses_);
        this->get_parameter("resolution", resolution);
        this->get_parameter("local_costmap.inflation_layer.inflation_radius", inflation_radius_);

        // merged map parameters
        merged_map.header.frame_id = "map";  // Use reference frame
        merged_map.info.resolution = resolution;  // Assume same resolution for both maps
        merged_map.info.origin.position.x = 0.0;
        merged_map.info.origin.position.y = 0.0;
        merged_map.info.origin.position.z = 0.0;
        merged_map.info.origin.orientation.w = 1.0;  // Identity quaternion

        border_size = inflation_radius_/resolution/2;

        // merged_map_cv = cv::Mat(2266,980, CV_8UC1, cv::Scalar(255));

        // Set QoS profile
        rclcpp::QoS qos_profile_reliable(rclcpp::KeepLast(1));
        qos_profile_reliable.transient_local();
        qos_profile_reliable.reliable();

        // Create publisher for the merged map
        merged_map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "map", qos_profile_reliable);

        obstacle_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle" , qos_profile_reliable);    

        // Create timer to control publishing of merged map
        timer_ptr_ = this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&MapMergerNode::timer_callback, this));

        // Subscribe to map topics dynamically based on the camera addresses
        for (const auto &camera_address : camera_addresses_)
        {
            std::string topic_name = "/map_" + camera_address;  // Generate topic name dynamically
            
            
            auto callback = [this, camera_address](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                this->mapCallback(msg, camera_address);
            };
            
            subscriptions_[camera_address] = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                topic_name, 10, callback);
            
            // Try to get maps static transform for 10secs
            if (!waitForTransform(camera_address,"map", "map_" + camera_address, std::chrono::seconds(10)))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get static transform for camera %s. Shutting down.", camera_address.c_str());
                rclcpp::shutdown();
                return;
            }
            

        }

        
    }

private:

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, const std::string &camera_address)
    {
        // Store the received map
        maps_[camera_address] = *msg;
        
    }
    void timer_callback(){

        // Check number of maps stored is same as number of cameras
        if (maps_.size() != camera_addresses_.size() && print_once == false){
            RCLCPP_INFO(this->get_logger(),"Maps stored: %ld but camera addresses: %ld", maps_.size(),camera_addresses_.size());
            print_once = true;
            return;
        }else if (maps_.size() != camera_addresses_.size() && print_once == true){
            return;
        }
        print_once = false;
        // Check if data size is same as map dimensions or dimensions is zero, ideally runs once
        if (merged_map.info.height*merged_map.info.width != merged_map.data.size() || merged_map.info.height == 0){
            get_merged_map_size();
        }
        // merge maps
        mergeMaps();
        obstacles();

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
        const auto& origin = merged_map.info.origin;
        const float resolution = merged_map.info.resolution;
        
        for (size_t y = 0; y < merged_map.info.height; ++y)
        {
            for (size_t x = 0; x < merged_map.info.width; ++x)
            {
                // Occupied cell value is 100
                if (merged_map.data[y * merged_map.info.width + x] == 100)
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

    void mergeMaps()
    {

        // Loop through stored maps and merges them
        for(const auto& map:maps_){
            transform_map(map.second,transforms_[map.first]);
        }
        // Add border to merged map
        // cv::Mat merged_map_cv_border;
        // cv::copyMakeBorder(merged_map_cv, merged_map_cv_border,1,1,1,1, cv::BORDER_CONSTANT, 0);
        // RCLCPP_INFO(this->get_logger(),"Size: %d, %d", merged_map_cv_border.rows, merged_map_cv_border.cols);
        // RCLCPP_INFO(this->get_logger(),"Size: %d, %d", merged_map_cv.rows, merged_map_cv.cols);
        // // cv::imshow("Merged Map", merged_map_cv_border);
        // cv::waitKey(1);
        
        // Fill in merged map parameters and publish     
        // cv::rectangle(merged_map_cv, cv::Rect(0,0,merged_map_cv.cols,merged_map_cv.rows),0,100);
        merged_map.header.stamp = this->now();
        merged_map.data.assign(merged_map_cv.begin<int8_t>(), merged_map_cv.end<int8_t>());
        merged_map_publisher_->publish(merged_map);
        

    }

    void get_merged_map_size() {

    
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();
        std::vector<geometry_msgs::msg::Point> points(4);
        for(auto& map:maps_){
            double h = map.second.info.height;
            double w = map.second.info.width;
        
            points[0].set__x(0.0); points[0].set__y(0.0); points[0].set__z(0.0);  // Bottom-left
            points[1].set__x(0.0); points[1].set__y(h); points[1].set__z(0.0);    // Top-left
            points[2].set__x(w); points[2].set__y(0.0); points[2].set__z(0.0);    // Bottom-right
            points[3].set__x(w); points[3].set__y(h); points[3].set__z(0.0);      // Top-right
        
            for (auto& pt : points) {
                geometry_msgs::msg::Point transformed_pt;
                // Apply static transform
                tf2::doTransform(pt, transformed_pt, transforms_[map.first]);
                // Keep track of min and max points
                min_x = std::min(min_x, transformed_pt.x);
                min_y = std::min(min_y, transformed_pt.y);
                max_x = std::max(max_x, transformed_pt.x);
                max_y = std::max(max_y, transformed_pt.y);
            }
        }
            // Transform all four corners and find the bounding box

    
        // Compute final width and height of the merged map and assign to merged_map
        double width = max_x - min_x;
        double height = max_y - min_y;

        // Set the merged map height and width
        merged_map.info.set__height(static_cast<unsigned int>(std::round(height+border_size)));
        merged_map.info.set__width(static_cast<unsigned int>(std::round(width+border_size)));
        merged_map.info.origin.position.set__x(min_x*resolution);
        merged_map.info.origin.position.set__y(min_y*resolution);
        // initialize merged map as single channel with all obstacles (100 as obstacle)
        merged_map_cv = cv::Mat(merged_map.info.height,merged_map.info.width, CV_8UC1, cv::Scalar(0));
        cv::rectangle(merged_map_cv, cv::Rect(0,0,merged_map_cv.cols,merged_map_cv.rows),100,1);
        RCLCPP_INFO(this->get_logger(),"Map: Width: %d , Height: %d", merged_map.info.width,merged_map.info.height);
    }
    
    void transform_map(const nav_msgs::msg::OccupancyGrid& map,
         geometry_msgs::msg::TransformStamped& transform)
    {
        // Define the grid's dimensions and resolution
        int width = map.info.width;
        int height = map.info.height;

        // Create a geometry_msgs Point to apply the transform
        geometry_msgs::msg::Point point_in_map;
        geometry_msgs::msg::Point point_in_world;
            // Iterate through each point in the map and transform it
            for (int i = 0; i < height; ++i) {
                for (int j = 0; j < width; ++j) {
              
                    point_in_map.x = j;
                    point_in_map.y = i;

                    // Transform the point
                    tf2::doTransform(point_in_map, point_in_world, transform);

                    // Get transformed coordinates
                    int new_x = static_cast<int>(point_in_world.x - merged_map.info.origin.position.x/resolution);
                    int new_y = static_cast<int>(point_in_world.y - merged_map.info.origin.position.y/resolution);

                    // Ensure indices are within bounds
                    if (new_x > border_size && new_x < merged_map_cv.cols-border_size && new_y > border_size && new_y < merged_map_cv.rows-border_size) {
                        // Get the occupancy value and set the transformed map
                        int index = i * width + j;
                        // Use AND logic to combine free space to existing merged map
                        if (merged_map_cv.at<uchar>(new_y,new_x) == 0 && map.data[index] == 0){
                            merged_map_cv.at<uchar>(new_y,new_x) = 0;
                        }else{
                            merged_map_cv.at<uchar>(new_y,new_x) = map.data[index];
                        }

                    }
                }
            }
        }
    // Function to wait for a transform to become available
    bool waitForTransform(const std::string &camera_address, const std::string &target_frame, const std::string &source_frame, std::chrono::seconds timeout)
    {
        auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok())
        {
            try
            {
                // Try to get the transform
                transforms_[camera_address] = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
                transforms_[camera_address].transform.translation.x = transforms_[camera_address].transform.translation.x / resolution;
                transforms_[camera_address].transform.translation.y = transforms_[camera_address].transform.translation.y / resolution;
                transforms_[camera_address].transform.translation.z = transforms_[camera_address].transform.translation.z / resolution;
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
 
    bool print_once = false;
    float resolution;
    double inflation_radius_;
    int border_size;
    // std::vector<int64_t> size;
    std::vector<cv::Mat> maps_cv;
    cv::Mat merged_map_cv ;
    // Store subscriptions dynamically for each camera address
    std::unordered_map<std::string, rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> subscriptions_;

    
    // Store received maps
    std::unordered_map<std::string, nav_msgs::msg::OccupancyGrid> maps_;
    
    // Publisher for the merged map
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_map_publisher_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    
    // List of camera addresses
    std::vector<std::string> camera_addresses_;

    // TF2 buffer and listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unordered_map<std::string,geometry_msgs::msg::TransformStamped> transforms_ ;
    geometry_msgs::msg::TransformStamped transform;
    nav_msgs::msg::OccupancyGrid merged_map;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher;
    sensor_msgs::msg::PointCloud2 obstacle;
    std::vector<geometry_msgs::msg::Point32> points;
    geometry_msgs::msg::Point32 pt;
    sensor_msgs::msg::PointField x_field, y_field, z_field;
    std::vector<sensor_msgs::msg::PointField> fields;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMergerNode>());
    rclcpp::shutdown();
    return 0;
}