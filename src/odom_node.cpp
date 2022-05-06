#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "rosproject1/Reset.h"

#include "rosproject1/odomConfig.h"
#include "dynamic_reconfigure/server.h"

class Odom {

    public:
        Odom() {

            if(!n.getParam("/x", x) || !n.getParam("/y", y) || !n.getParam("/theta", theta))
            {
                ROS_WARN("Odometry Initialization: Some parameters were not found");
                x = 0;
                y = 0;
                theta = 0;
            }
            

            /*
            this->x = x;
            this->y = y;
            this->theta = theta;*/


            cmd_vel_sub = n.subscribe("cmd_vel", 1000, &Odom::cmd_vel_sub_callback, this);
            odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
            service = n.advertiseService<Odom, rosproject1::Reset::Request, rosproject1::Reset::Response>("reset", &Odom::reset_callback, this);

            dynamic_reconfigure::Server<rosproject1::odomConfig>::CallbackType f;
            f = boost::bind(&Odom::set_integration, this, _1, _2);


            dynServer.setCallback(f );
        }

        void set_integration(rosproject1::odomConfig &config, uint32_t level) {

            int32_t int_method = config.integration_method;

            if(int_method == 0)
                method = EULER;
            else
                method = RK;

            ROS_INFO("Reconfigure Request: %d - Level %d", config.integration_method, level);
        }

        void main_loop() {
            ros::Rate loop_rate(10);

            this->prev_stamp = ros::Time::now();

            while (ros::ok()) {

            ros::spinOnce();
            loop_rate.sleep();
            }
        }

        bool reset_callback(rosproject1::Reset::Request &req, rosproject1::Reset::Response &res) {
            x = req.x;
            y = req.y;
            theta = req.theta;

            res.result = 1;
            return true;
        }

        void cmd_vel_sub_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
            double deltatime = msg->header.stamp.sec - prev_stamp.sec + (msg->header.stamp.nsec - prev_stamp.nsec) / 1e9;

            prev_stamp = msg->header.stamp;


            if(deltatime < 1) {
                std::stringstream ss;

                

                double vx_robot = msg->twist.linear.x;
                double vy_robot = msg->twist.linear.y;
                double vtheta = msg->twist.angular.z;

                double midpoint_increment;

                if(method == RK)
                    midpoint_increment = vtheta * deltatime / 2;
                else
                    midpoint_increment = 0;


                double vx = vx_robot * cos(theta + midpoint_increment) -  vy_robot * sin(theta + midpoint_increment);
                double vy = vx_robot * sin(theta + midpoint_increment) + vy_robot * cos(theta + midpoint_increment);


                x +=  vx* deltatime;
                y +=  vy* deltatime;
                theta += vtheta * deltatime;

                ss << "Displacement: " << sqrt(pow(vx*deltatime,2)+pow(vy*deltatime,2));
                ROS_INFO_STREAM(ss.str());

                nav_msgs::Odometry odom;

                odom.header.stamp = msg->header.stamp;
                odom.child_frame_id = "base_link";
                odom.header.frame_id = "odom";

                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = 0;

                tf2::Quaternion orientation;
                orientation.setRPY(0, 0, theta);

                odom.pose.pose.orientation.w = orientation.w();
                odom.pose.pose.orientation.x = orientation.x();
                odom.pose.pose.orientation.y = orientation.y();
                odom.pose.pose.orientation.z = orientation.z();

                odom.twist.twist.linear.x = vx_robot;
                odom.twist.twist.linear.y = vy_robot;
                odom.twist.twist.linear.z = 0;

                odom.twist.twist.angular.x = 0;
                odom.twist.twist.angular.y = 0;
                odom.twist.twist.angular.z = vtheta;

                for(int i = 0; i < 6; i++)
                    odom.pose.covariance[i,i] = 1;

                transform_stamped.header.stamp = msg->header.stamp;
                transform_stamped.header.frame_id = "odom";
                transform_stamped.child_frame_id = "base_link";

                transform_stamped.transform.translation.x = x;
                transform_stamped.transform.translation.y = y;
                transform_stamped.transform.translation.z = 0.0;
                transform_stamped.transform.rotation.w = orientation.w();
                transform_stamped.transform.rotation.x = orientation.x();
                transform_stamped.transform.rotation.y = orientation.y();
                transform_stamped.transform.rotation.z = orientation.z();

                odom_pub.publish(odom);
                br.sendTransform(transform_stamped);
            }
        }

    private:
    nav_msgs::Odometry odometry;
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher odom_pub;
    ros::Time prev_stamp;

    enum integration_method{EULER=0, RK=1};

    integration_method method;

    ros::ServiceServer service;

    dynamic_reconfigure::Server<rosproject1::odomConfig> dynServer;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform_stamped;


    double x;
    double y;
    double theta;

};



int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_node");

    Odom odom_node;

    odom_node.main_loop();

    return 0;
}