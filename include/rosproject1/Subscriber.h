#define SUBSCRIBER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3.h"

class Subscriber {
public:
  Subscriber(); 
  void main_loop();
  void velCallback(const sensor_msgs::JointState::ConstPtr& msg);

private:
  //member variables, visible only IN the class
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher vel_pub;
  geometry_msgs::Vector3 linear;
  geometry_msgs::Vector3 angular;
  ros::Time prev_stamp;
  int count;
};