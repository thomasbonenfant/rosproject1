#include "ros/ros.h"
#include "rosproject1/WheelsSubscriber.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "wheels_sub");
  
  //calling the contructor
  WheelsSubscriber wheels_sub;

  wheels_sub.main_loop();

  return 0;
}
