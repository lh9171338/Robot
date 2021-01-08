#include <ros/ros.h>
#include "car.hpp"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "car");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Create Car class object
  robot::Car car;

  if(car.setup(nh, nh_priv))
  {
    // Initial car successfully
    car.spin();
  }

  return 0;
}