#define WHEELSSUBSCRIBER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3.h"

class WheelsSubscriber {
public:
  WheelsSubscriber(); 
  void main_loop();
  void velCallback(const sensor_msgs::JointState::ConstPtr& msg);
  static constexpr float radius = 0.07; //radius of wheels
  static constexpr int N = 42; //encoder CPR
  static constexpr double T = 5; //gear ratio
  static constexpr double l = 0.2; //wheel position along x (+-l)
  static constexpr double w = 0.169; //wheel position along y (+-w)
  static constexpr double multFactor = (1/(N*T)) * 2 * M_PI; 

private:
  //member variables, visible only IN the class
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher vel_pub;
  geometry_msgs::Vector3 linear;
  geometry_msgs::Vector3 angular;
  ros::Time prev_stamp;
  std::vector<double> prev_position;
};
