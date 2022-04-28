#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>

#include "rosproject1/WheelsSpeeds.h"

class VelSubscriber {
public:
	static constexpr float radius = 0.07; //radius of wheels
  static constexpr double l = 0.2; //wheel position along x (+-l)
  static constexpr double w = 0.169; //wheel position along y (+-w)
  static constexpr double T = 5; //gear ratio
  static constexpr double multFactor = (60 * T) / radius;
  
  VelSubscriber() {
  	//this specifies also in which class the callback is defined
	  this->sub = this->n.subscribe("cmd_vel", 1000, &VelSubscriber::wheelsCallback, this);
	  this->wheels_pub = this->n.advertise<rosproject1::WheelsSpeeds>("wheels_rpm", 1000);
  } 

  void main_loop() {
  	ros::Rate loop_rate(100);

		while (ros::ok()) {
			ros::spinOnce();
			loop_rate.sleep();
		}
  }

  // this callback is gonna publish wheels velocities
	void wheelsCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
		std::stringstream ss;
		ss << "I heard timestamp sec: " << msg->header.stamp.sec << std::endl;
	  ss << "and nanosec: " << msg->header.stamp.nsec << std::endl;
	  
		//create a new message of type WheelsSpeeds to be filled
		rosproject1::WheelsSpeeds speeds_msg;

		//setting the header
		//speeds_msg.header.seq = msg->header.seq;
		ss << "I refer to seq: " << msg->header.seq << std::endl;
		speeds_msg.header.stamp = msg->header.stamp;
		speeds_msg.header.frame_id = msg->header.frame_id;

		ss << "Received omegaz: " << msg->twist.angular.z << std::endl;
	  ss << "Received vx: " << msg->twist.linear.x << std::endl;
	  ss << "Received vy: " << msg->twist.linear.y << std::endl;

		speeds_msg.rpm_fl = (((- VelSubscriber::l - VelSubscriber::w) * msg->twist.angular.z) + 
		msg->twist.linear.x - msg->twist.linear.y) * VelSubscriber::multFactor;
		speeds_msg.rpm_fr = (((VelSubscriber::l + VelSubscriber::w) * msg->twist.angular.z) + 
		msg->twist.linear.x + msg->twist.linear.y) * VelSubscriber::multFactor;
		speeds_msg.rpm_rl = (((VelSubscriber::l + VelSubscriber::w) * msg->twist.angular.z) + 
		msg->twist.linear.x - msg->twist.linear.y) * VelSubscriber::multFactor;
		speeds_msg.rpm_rr = (((- VelSubscriber::l - VelSubscriber::w) * msg->twist.angular.z) + 
		msg->twist.linear.x + msg->twist.linear.y) * VelSubscriber::multFactor;

		ss << "Computed rpm for front left wheel: " << speeds_msg.rpm_fl << std::endl;
	  ss << "Computed rpm for front right wheel: " << speeds_msg.rpm_fr << std::endl;
	  ss << "Computed rpm for rear left wheel: " << speeds_msg.rpm_rl << std::endl;
	  ss << "Computed rpm for rear right wheel: " << speeds_msg.rpm_rr << std::endl;
	  ROS_INFO_STREAM(ss.str());

	  this->wheels_pub.publish(speeds_msg);
	}

private:
  //member variables
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher wheels_pub;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vel_sub");
  
  //calling the contructor
  VelSubscriber vel_sub;

  vel_sub.main_loop();

  return 0;
}
