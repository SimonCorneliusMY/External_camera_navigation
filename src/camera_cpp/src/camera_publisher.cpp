/* 240219 Simon
Publishes camera image to ROS2, by default tries to open default webcam
Default resolution is set at 720p
Parameters taken
view_feed: camera view
camera_address: I use cv::VideoCapture(), the input tested is string "0" and "http://192.168.0.103:81/stream"
name: the appended name to the camera, default 0

TODO when closing the node error message accidentaly prompted
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <chrono>
#include <string>
#include <iostream>
#include <curl/curl.h>

class WebcamPublisher : public rclcpp::Node {
public:
    WebcamPublisher()
        : Node("webcam_publisher") {

        this->declare_parameter<bool>("view_feed", false);
        // this->declare_parameter<std::string>("camera_address", "http://192.168.0.101:81/stream");
        this->declare_parameter<std::string>("camera_address", "udp://@192.168.0.102:5000");
        this->declare_parameter<std::string>("name","0");
        

 
        rclcpp::QoS profile(rclcpp::KeepLast(10));
        profile.reliable();

        this->get_parameter("name",name);
        if(!this->get_parameter("camera_address",camera_address)){
            RCLCPP_ERROR(this->get_logger(), "Failed to get camera address");
        }
        std::string pipeline = 
        "udpsrc port=" + camera_address +" buffer-size=65536 ! "  // Increased buffer size
        "application/x-rtp,media=video,encoding-name=JPEG,payload=26 ! "
        "rtpjpegdepay ! jpegdec ! "
        "queue max-size-buffers=2 ! "  // Small queue for minimal latency
        "videoconvert ! videorate ! "
        "video/x-raw,format=BGR,framerate=30/1 ! "
        "appsink sync=false drop=true";

        // Open the default webcam. TODO if use http camera and http camera not available it gets stuck even pressing ctrl c fails to kill it.
        while(!cap_.isOpened() && rclcpp::ok()){
            try{
                RCLCPP_INFO(this->get_logger(),"%d", cap_.open(camera_address,cv::CAP_GSTREAMER));
                if(camera_address.size() == 1 && cap_.open(std::stoi(camera_address), cv::CAP_V4L2)){
                    RCLCPP_INFO(this->get_logger(), "%s", camera_address.c_str());
                    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));  // MJPEG format
                    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);  // Set width to 1280px
                    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720); // Set height to 720px
                }
                
                else if(cap_.open(pipeline,cv::CAP_GSTREAMER)){
                    RCLCPP_INFO(this->get_logger(), "UDP stream port number: %s", camera_address.c_str());
                }

                else if(cap_.open(camera_address,cv::CAP_FFMPEG)){
                    std::string url = camera_address.substr(0,camera_address.find_last_of(":"));
                    http_set_resolution(url,"13");
                }else{
                    RCLCPP_WARN(this->get_logger(), "Failed to open camera, address: %s.", camera_address.c_str());
                    sleep(5);
                }

            }catch(std::exception& e){
                RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
            }


        }

        // Create the image publisher and FPS publisher
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_"+name + "/image_raw", profile);
        fps_publisher_ = this->create_publisher<std_msgs::msg::Int32>("fps_"+name, 10);

        // Set up a timer to run the callback at 10Hz (every 100 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&WebcamPublisher::timer_callback, this));

        // Capture the initial time
        last_time_ = std::chrono::steady_clock::now();
    }

private:
// so you can set the camera's settings in the arduino file itself, hence WriteCallback and http_set_resolution is obsolete 250424
    // chatgpt handywork
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
        ((std::string*)userp)->append((char*)contents, size * nmemb);
        return size * nmemb;
    }
    // chatgpt sonnet
    void http_set_resolution(std::string& url, std::string index){
        CURL* curl;
        CURLcode res;
        std::string readBuffer;

        // Initialize libcurl
        curl_global_init(CURL_GLOBAL_DEFAULT);
        curl = curl_easy_init();


        if (curl) {
            // You can get the index needed by opening the camera in a browser then right click and inspect, then network tab.
            // When you change the resolution, you will be able to see the request it send
            // The URL to which the GET request will be made
            url = url + "/control?var=framesize&val=" + index;
            RCLCPP_INFO(this->get_logger(), "%s",url.c_str());
            // Set the URL
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

            // Set the function to handle the response
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

            // Perform the GET request
            res = curl_easy_perform(curl);

            if (res != CURLE_OK) {
                std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            } else {
                // Print the response if the request was successful
                std::cout << "Response: " << readBuffer << std::endl;
            }

            // Clean up
            curl_easy_cleanup(curl);
        }

        // Clean up global resources used by libcurl
        curl_global_cleanup();
    }
    void timer_callback() {
        auto now = std::chrono::steady_clock::now();
        // Capture a frame from the webcam
        bool view_feed = this -> get_parameter("view_feed").get_value<bool>();


        cap_ >> frame;


        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to capture image.");
            return;
        }

        // Convert the OpenCV image to ROS Image message
        auto ros_image = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        auto image_ptr = std::make_shared<sensor_msgs::msg::Image>(*ros_image);
  
        // Publish the image
        image_ptr->header.stamp = rclcpp::Clock().now();
        image_publisher_->publish(*image_ptr);

        if (view_feed == true){
            cv::imshow("Camera feed"+name, frame);
            cv::waitKey(1);
        }
        

        // Log the first publish
        if (first_publish_) {
            RCLCPP_INFO(this->get_logger(), "Publishing webcam feed...");
            first_publish_ = false;
        }

        // Calculate FPS based on the time elapsed since the last frame
        
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
    std::string camera_address, name;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebcamPublisher>());
    rclcpp::shutdown();
    return 0;
}
