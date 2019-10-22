#include <ros/ros.h>
#include "reduce.h"

int main(int argc, char** argv)
{
  // start node
  ros::init(argc, argv, "reduce_resolution_node");

  // instantiate the reduce_resolution::ReduceResolution class
  reduce_resolution::ReduceResolution reduce_res(0.5);

  ros::spin();
  return 0;
}