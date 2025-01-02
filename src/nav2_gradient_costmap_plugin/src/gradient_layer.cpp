/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/

#include "nav2_gradient_costmap_plugin/gradient_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include <opencv2/opencv.hpp>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_gradient_costmap_plugin1
{

GradientLayer::GradientLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
GradientLayer::onInitialize()
{
  RCLCPP_INFO(rclcpp::get_logger("nav2_gradient_costmap_plugin"), "Received map with size: ");
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + "." + "enabled", enabled_);

  map_subscription_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&GradientLayer::mapCallback, this, std::placeholders::_1) // the topic name /map for absolute without namespace while map includes namespace
  );

  need_recalculation_ = true;
  current_ = true;
  cv::namedWindow("Image Display", cv::WINDOW_NORMAL);
  cv::namedWindow("Resized", cv::WINDOW_NORMAL);

}

void GradientLayer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = msg;
  // Access the map dimensions
  width = msg->info.width;
  height = msg->info.height;

}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
GradientLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  robot_x_ = robot_x/0.002942643;
  robot_y_ = robot_y/0.002942643;

  // RCLCPP_INFO(rclcpp::get_logger("nav2_gradient_costmap_plugin"), "Pose: %f, %f, Pose 2: %f, %f", robot_x_,robot_y_,robot_x,robot_y);
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);

    // RCLCPP_INFO(rclcpp::get_logger("nav2_gradient_costmap_plugin"), "Size: %f, %f, %f, %f", *min_x,*min_y,*max_x,*max_y);
    
  }
}

// The method is called when footprint was changed.
// Here it just resets need_recalculation_ variable.
void
GradientLayer::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i, int max_j)
{
  
  if (!enabled_) {
    return;
  }
  
  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  // unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  // RCLCPP_INFO(rclcpp::get_logger("nav2_gradient_costmap_plugin"), "Size: %d, %d, %d, %d", min_i,min_j,max_i,max_j);
  // //determining the boundary points of the local costmap based on the global map
  // min_i = robot_x_ - 784 / 2;
  // max_i = robot_x_ + 784 / 2;
  // min_j = robot_y_ - 784 / 2;
  // max_j = robot_y_ + 784 / 2;

  

  // for (int j = 0; j < 784; j++) {
  //   for (int i = 0; i < 784; i++) {

  //       // Get the corresponding 1D index for (i, j) position
  //       int index = master_grid.getIndex(i, j);
  //       // setting the bounds of costmap update
  //       if (j+min_j >=0 && j+min_j <= height && i + min_i >= 0 && i+min_i<= width){
  //         master_array[index] = latest_map_->data[((j+min_j) * width) + i + min_i];  
  //       }
  //   }
  // }  
  // determine the boundary of the global map
  min_x = robot_x_ - 784 / 2;
  max_x = robot_x_ + 784 / 2;
  min_y = robot_y_ - 784 / 2;
  max_y = robot_y_ + 784 / 2;

  // determining the local costmap boundary to update
  if(min_x < 0){
    min_i = -min_x;
  }else{
    min_i = 0;
  }

  if(min_y < 0){
    min_j = -min_y;
  }else{
    min_j = 0;
  }  
  if (max_x > width){
    max_i = 784 - max_x + width;
  }else{
    max_i = 784;
  }
  if (max_y > height){
    max_j = 784 - max_y + height;
  }else{
    max_j = 784;
  }

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {

        // Get the corresponding 1D index for (i, j) position
        int index = master_grid.getIndex(i, j);
        // setting the bounds of costmap update

        master_array[index] = latest_map_->data[((j+min_y) * width) + i + min_x];  

    }
  } 

}

}  // namespace nav2_gradient_costmap_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_gradient_costmap_plugin1::GradientLayer, nav2_costmap_2d::Layer)