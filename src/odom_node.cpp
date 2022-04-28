#include "ros/ros.h"
#include "rosproject1/Odom.h"
#include "tf2/LinearMath/Quaternion.h"

Odom::Odom(double x, double y, double theta) {

    this->x = x;
    this->y = y;
    this->theta = theta;


    cmd_vel_sub = n.subscribe("cmd_vel", 1000, &Odom::cmd_vel_sub_callback, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
}

void Odom::main_loop() {
    ros::Rate loop_rate(10);

    this->prev_stamp = ros::Time(0);

	while (ros::ok()) {

	ros::spinOnce();
	loop_rate.sleep();
	}
}

void Odom::cmd_vel_sub_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    double deltatime = msg->header.stamp.sec - prev_stamp.sec + (msg->header.stamp.nsec - prev_stamp.nsec) / 1e9;
    prev_stamp = msg->header.stamp;

    double vx = msg->twist.linear.x;
    double vy = msg->twist.linear.y;
    double vtheta = msg->twist.angular.z;

    //integrate with euler method
    x += vx * deltatime;
    y += vy * deltatime;
    theta += vtheta * deltatime;

    //publish odometry
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, theta);

    odom.pose.pose.orientation.w = orientation.w();
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();

    odom_pub.publish(odom);
    

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_node");

    Odom odom_node = Odom(0,0,0);

    odom_node.main_loop();

    return 0;
}