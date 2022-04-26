#include "ros/ros.h"
#include "rosproject1/VelSubscriber.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "vel_sub");
  
  //calling the contructor
  VelSubscriber vel_sub;

  vel_sub.main_loop();

  return 0;
}
