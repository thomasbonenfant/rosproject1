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
  this->sub = this->n.subscribe("wheel_states", 1000, &Subscriber::velCallback, this);
  this->vel_pub = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  /*
  const float radius = 0.07; //radius of wheels
  const int N = 42; //encoder CPR
  const double T = 5; //gear ratio
  const double l = 0.2; //wheel position along x (+-l)
  const double w = 0.169; //wheel position along y (+-w)
  const double multFactor = (1/(N*T)) * 2 * M_PI; */
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

  this->prev_stamp = ros::Time(0);

	//this->count = 0;

	while (ros::ok()) {

	ros::spinOnce();
	loop_rate.sleep();
	//++count;
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

  //also time expressed in double to be compatible with position
  geometry_msgs::TwistStamped vel_msg;
  double deltaTime;
  std::vector<double> deltaTicks;
  std::vector<double> wheelsVel(4);
  
  if (this->prev_stamp != ros::Time(0)) {

    ss.str("");
    ss << std::setprecision(9); //to see only 3 decimals
    ss << "Current seq: " << vel_msg.header.seq << std::endl;

    // computation of deltaTime in nsec
    if (this->prev_stamp.sec == msg->header.stamp.sec) {
      deltaTime = msg->header.stamp.nsec - this->prev_stamp.nsec;
    }
    else
      deltaTime = (1000000000 - this->prev_stamp.nsec) + msg->header.stamp.nsec;
    if (msg->header.stamp.sec - this->prev_stamp.sec > 1)
      ROS_INFO("WARNING: difference of time is more than one second ");

    ss << "Computed deltaTime in nsec: " << deltaTime << std::endl;

    // conversion of deltaTime from nsec to sec
    deltaTime = deltaTime/1000000000;

    ss << "Computed deltaTime in sec: " << deltaTime << std::endl;
    
    // computation of deltaTicks
    std::transform((msg->position).begin(), (msg->position).end(), (this->prev_position).begin(), 
      std::back_inserter(deltaTicks), [](double l, double r) { return fabs(l - r);});

    ss << "Computed deltaTicks in x: " << deltaTicks.at(0) << std::endl;
    ss << "From new position: " << msg->position.at(0) << std::endl;
    ss << "And prev position: " << this->prev_position.at(0) << std::endl;

    ss << "Computed deltaTicks in y: " << deltaTicks.at(1) << std::endl;
    ss << "From new position: " << msg->position.at(1) << std::endl;
    ss << "And prev position: " << this->prev_position.at(1) << std::endl;
    
    // dividing deltaTicks for deltaTime
    transform(deltaTicks.begin(), deltaTicks.end(), deltaTicks.begin(), 
      [deltaTime](double &c){ return c/deltaTime;});

    ss << "DeltaTicks/DeltaTime: " << deltaTicks.at(0) << std::endl;
    
    // finding wheels velocity multiplying deltaTicks with multFactor
    wheelsVel = deltaTicks;
    double mult = Subscriber::multFactor;
    transform(wheelsVel.begin(), wheelsVel.end(), wheelsVel.begin(), 
      [mult](double &c){ return c*mult;});

    ss << "WheelsVel in x: " <<wheelsVel.at(0) << std::endl;
    ss << "WheelsVel in y: " <<wheelsVel.at(1) << std::endl;
    ss << "WheelsVel in z: " <<wheelsVel.at(2) << std::endl;

    // computing linear and angular speeds from given system info and wheels velocities
    
    this->angular.z = (wheelsVel.at(1) - wheelsVel.at(2) + wheelsVel.at(3) - wheelsVel.at(0))*
    Subscriber::radius/(4*(Subscriber::l + Subscriber::w));
    this->linear.x = (wheelsVel.at(0) + wheelsVel.at(1) + wheelsVel.at(2) + wheelsVel.at(3))*Subscriber::radius/4;
    this->linear.y = (wheelsVel.at(1) + wheelsVel.at(2) - wheelsVel.at(3)- wheelsVel.at(0))*Subscriber::radius/4;
    
    //set values
    vel_msg.twist.linear = this->linear;
    vel_msg.twist.angular = this->angular;
    //set the header
    vel_msg.header.seq = msg->header.seq;
    vel_msg.header.frame_id = msg->header.frame_id;
    //when we reach a timestamp in the msg, we should use it
    //also for the published msg as it refers also to that time
    vel_msg.header.stamp = msg->header.stamp;

    
    
    ss << "The computed linear velocity is: " << vel_msg.twist.linear.x << " along x axis," << std::endl;
    ss << vel_msg.twist.linear.y << " along y axis," << std::endl;
    ss << vel_msg.twist.linear.z << " along z axis," << std::endl;
    ss << "The computed angular velocity is: " << vel_msg.twist.angular.x << " along x axis," << std::endl;
    ss << vel_msg.twist.angular.y << " along y axis," << std::endl;
    ss << vel_msg.twist.angular.z << " along z axis" << std::endl;
    ROS_INFO_STREAM(ss.str());
    
    this->vel_pub.publish(vel_msg);
  }

  this->prev_stamp = msg->header.stamp;
  this->prev_position = msg->position;
}