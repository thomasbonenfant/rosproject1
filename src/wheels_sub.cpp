#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"

#include "rosproject1/calibrationConfig.h"
#include "dynamic_reconfigure/server.h"

class WheelsSubscriber {
public:

  WheelsSubscriber() {
    //this specifies also in which class the callback is defined
    this->sub = this->n.subscribe("wheel_states", 1000, &WheelsSubscriber::velCallback, this);
    this->vel_pub = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);

    dynamic_reconfigure::Server<rosproject1::calibrationConfig>::CallbackType f;
    f = boost::bind(&WheelsSubscriber::set_parameters, this, _1, _2);

    dynServer.setCallback(f);

  }

  void set_parameters(rosproject1::calibrationConfig &config, uint32_t level) {

  radius = config.radius;
  N = config.N;
  l = config.l;
  w = config.w;
  multFactor = (1/(N*T)) * 2 * M_PI;

  }

  void main_loop() {
    ros::Rate loop_rate(10);

    this->prev_stamp = ros::Time(0);

    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  void velCallback(const sensor_msgs::JointState::ConstPtr& msg) {

    //also time expressed in double to be compatible with position
    geometry_msgs::TwistStamped vel_msg;
    double deltaTime;
    std::vector<double> deltaTicks;
    std::vector<double> wheelsVel(4);
    
    if (this->prev_stamp != ros::Time(0)) {

      // computation of deltaTime in nsec
      deltaTime = msg->header.stamp.sec - prev_stamp.sec + ((msg->header.stamp.nsec - prev_stamp.nsec) / 1e9);

      // computation of deltaTicks
      std::transform((msg->position).begin(), (msg->position).end(), (this->prev_position).begin(), 
        std::back_inserter(deltaTicks), [](double l, double r) { return l - r;});

      // dividing deltaTicks for deltaTime
      transform(deltaTicks.begin(), deltaTicks.end(), deltaTicks.begin(), 
        [deltaTime](double &c){ return c/deltaTime;});

      // finding wheels velocity multiplying deltaTicks with multFactor
      wheelsVel = deltaTicks;
      double mult = WheelsSubscriber::multFactor;
      transform(wheelsVel.begin(), wheelsVel.end(), wheelsVel.begin(), 
        [mult](double &c){ return c*mult;});

      // computing linear and angular speeds from given system info and wheels velocities
      this->angular.z = (wheelsVel.at(1) - wheelsVel.at(2) + wheelsVel.at(3) - wheelsVel.at(0))*
      WheelsSubscriber::radius/(4*(WheelsSubscriber::l + WheelsSubscriber::w));
      this->linear.x = (wheelsVel.at(0) + wheelsVel.at(1) + wheelsVel.at(2) + wheelsVel.at(3))*WheelsSubscriber::radius/4;
      this->linear.y = (wheelsVel.at(1) + wheelsVel.at(2) - wheelsVel.at(3)- wheelsVel.at(0))*WheelsSubscriber::radius/4;
      
      //set values
      vel_msg.twist.linear = this->linear;
      vel_msg.twist.angular = this->angular;
      
      // header seq updates autonomously
      vel_msg.header.frame_id = msg->header.frame_id;
      //published msg with the same timestamp of received msg since it refers also to that time
      vel_msg.header.stamp = msg->header.stamp;
      
      this->vel_pub.publish(vel_msg);
    }
    this->prev_stamp = msg->header.stamp;
    this->prev_position = msg->position;
  }
   
private:
  //member variables, visible only IN the class
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher vel_pub;
  geometry_msgs::Vector3 linear;
  geometry_msgs::Vector3 angular;
  ros::Time prev_stamp;
  std::vector<double> prev_position;


  dynamic_reconfigure::Server<rosproject1::calibrationConfig> dynServer;


  float radius = 0.07; //radius of wheels
  int N = 42; //encoder CPR
  const double T = 5; //gear ratio
  double l = 0.2; //wheel position along x (+-l)
  double w = 0.169; //wheel position along y (+-w)
  double multFactor = (1/(N*T)) * 2 * M_PI;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "wheels_sub");
  
  //calling the contructor
  WheelsSubscriber wheels_sub;

  wheels_sub.main_loop();

  return 0;
}
