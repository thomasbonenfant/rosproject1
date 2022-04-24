#include "ros/ros.h"
#include "rosproject1/Subscriber.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  
  //calling the contructor
  Subscriber my_sub;

  my_sub.main_loop();

  return 0;
}
