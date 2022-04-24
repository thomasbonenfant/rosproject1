#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
//useful link: https://la.mathworks.com/help/ros/ug/work-with-basic-ros-messages.html
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>

#include "rosproject1/Subscriber.h"


//constructor of the classs, initializations
Subscriber::Subscriber() {
  //this specifies also in which class the callback is defined
  this->sub = this->n.subscribe("wheel_states", 1, &Subscriber::velCallback, this);
  this->vel_pub = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);
}

void Subscriber::main_loop() {
	ros::Rate loop_rate(10);
	// instead of std_msgs I have to publish v and omega as topic cmd_vel
	// of type geometry_msgs/TwistStamped

	//setting sample values

	this->linear.x = 1.0;
  this->linear.y = 1.0;
  this->linear.z = 0.0;
	this->angular.x = 0.0;
  this->angular.y = 0.0;
  this->angular.z = 1.0;

	this->count = 0;

	while (ros::ok()) {

	ros::spinOnce();
	loop_rate.sleep();
	++count;
	}
}

void Subscriber::velCallback(const sensor_msgs::JointState::ConstPtr& msg) {

  //code to print received message infos
  std::string name;
  for (auto i: msg->name) {
  		name.append(i.c_str());
		name.append(" ");
	}
  std::stringstream ss;
  ss << std::setprecision(3); //to see only 3 decimals
  ss << "I heard timestamp sec: " << msg->header.stamp.sec << std::endl;
  ss << "and nanosec: " << msg->header.stamp.nsec << std::endl;
  ss << "wheels: " << name.c_str() << std::endl;
  ss << "position: ";
  std::copy(msg->position.begin(), msg->position.end(), std::ostream_iterator<double>(ss, " "));
  ss << std::endl;
  ss << "velocity: ";
  std::copy(msg->velocity.begin(), msg->velocity.end(), std::ostream_iterator<double>(ss, " "));
  ss << std::endl;
  ROS_INFO_STREAM(ss.str());

  float deltaTime, deltaTicks;

  if (this->prev_stamp != NULL) {
    deltaTime = this->prev_stamp - msg->header.stamp;
  }
  else
    deltaTime = msg->header.stamp.nsec;

  geometry_msgs::TwistStamped vel_msg;
  //this.angular = msg->position * ...
  //this.linear = msg->position * ...
  //set values
  vel_msg.twist.linear = this->linear;
  vel_msg.twist.angular = this->angular;
  //set the header
  vel_msg.header.seq = this->count;
  //when we reach a timestamp in the msg, we should use it
  //as the published msg refers also to that time
  vel_msg.header.stamp = msg->header.stamp;

  ROS_INFO("Current id: %d", vel_msg.header.seq);
  this->vel_pub.publish(vel_msg);

  this->prev_stamp = msg->header.stamp;
}