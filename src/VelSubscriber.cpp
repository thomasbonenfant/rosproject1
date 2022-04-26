#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>

#include "rosproject1/VelSubscriber.h"

//constructor of the classs, initializations
VelSubscriber::VelSubscriber() {

  //this specifies also in which class the callback is defined
  this->sub = this->n.subscribe("cmd_vel", 1000, &VelSubscriber::wheelsCallback, this);
  this->wheels_pub = this->n.advertise<geometry_msgs::TwistStamped>("wheels_rpm", 1000);
}

void VelSubscriber::main_loop() {

	ros::Rate loop_rate(10);

	while (ros::ok()) {

	ros::spinOnce();
	loop_rate.sleep();
	}
}

void VelSubscriber::wheelsCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {

  //to be implemented
  
}