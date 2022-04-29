#define ODOM_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"


class Odom {
    public:
        Odom(double x, double y, double theta);
        void main_loop();
        void cmd_vel_sub_callback(const geometry_msgs::TwistStamped::ConstPtr& msg); //subcriber's callback on cmd_vel


    private:
        nav_msgs::Odometry odometry;
        ros::NodeHandle n;
        ros::Subscriber cmd_vel_sub;
        ros::Publisher odom_pub;
        ros::Time prev_stamp;

        tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform_stamped;


        double x;
        double y;
        double theta;
};
