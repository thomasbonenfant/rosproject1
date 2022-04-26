#define VELSUBSCRIBER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

class VelSubscriber {
public:
  VelSubscriber(); 
  void main_loop();
  // this callback is gonna publish wheels velocities
  void wheelsCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  static constexpr float radius = 0.07; //radius of wheels
  static constexpr double l = 0.2; //wheel position along x (+-l)
  static constexpr double w = 0.169; //wheel position along y (+-w)

private:
  //member variables
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher wheels_pub;
};
