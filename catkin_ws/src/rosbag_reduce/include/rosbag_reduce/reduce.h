#pragma once

// libraries
#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// messages
#include "sensor_msgs/Image.h"

namespace reduce_resolution {

  class ReduceResolution
  {
  public:
    ReduceResolution(double resize_ratio);

  private:
    // Image Transport Publishers and Subscribers
    void CallbackVideo(const sensor_msgs::ImageConstPtr& data);
    
    // ROS
    ros::NodeHandle nh_;
    image_transport::Subscriber sub_video_;
    image_transport::Publisher pub_video_;

    bool init_;
    cv_bridge::CvImagePtr msg_;
    cv::Mat frame_;
    cv::Mat frame_reduced_;
    cv::Size sd_resolution_;
    double resize_ratio_;
  };

}
