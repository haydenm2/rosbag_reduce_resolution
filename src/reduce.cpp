#include "reduce.h"

namespace reduce_resolution {

ReduceResolution::ReduceResolution(double resize_ratio)
{
  // create a private node handle for use with param server
  ros::NodeHandle nh_private("~");

  // ROS communication
  image_transport::ImageTransport it(nh_);
  sub_video_ = it.subscribe("video", 100, &ReduceResolution::CallbackVideo,  this);
  pub_video_ = it.advertise("video/image_raw", 1);
  init_ = true;
  resize_ratio_ = resize_ratio;
}

// ----------------------------------------------------------------------------

void ReduceResolution::CallbackVideo(const sensor_msgs::ImageConstPtr& data)
{
  
  // Convert incoming image data to OpenCV format
  msg_ = cv_bridge::toCvCopy(data, "bgr8");
  frame_ = msg_->image;

  if(init_)
  {
    // Define reduced image size parameters
    sd_resolution_.width = frame_.size().width*resize_ratio_;
    sd_resolution_.height = frame_.size().height*resize_ratio_;    
    
    init_ = false;
  }

  // Reduce size of image message
  cv::resize(frame_, frame_reduced_, sd_resolution_, 0, 0, cv::INTER_AREA);
  msg_->image = frame_reduced_;

  // Publish reduced image
  pub_video_.publish(msg_->toImageMsg());

}

} // namespace reduce_resolution