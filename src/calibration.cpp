#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <sstream>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calibration_node");
    ros::start();

    ros::Subscriber sub_gt_pose;

    ros::Subscriber sub_wheel_states;


    ros::Rate loop_rate(10);

    geometry_msgs::PoseStampedConstPtr gt_pose;
    sensor_msgs::JointStateConstPtr wheels_state;

    float radius = 0.07;
    int N = 42; //encoder CPR
    double const T = 5; //gear ratio
    double l = 0.2; //wheel position along x (+-l)
    double w = 0.169; //wheel position along y (+-w)

    double multFactor = (1/(N*T)) * 2 * M_PI;
    double deltaTime;

    
    ros::Time prev_stamp = ros::Time::now();
    std::vector<double> deltaTicks(4);
    std::vector<double> prev_ticks(4);

    std::vector<double> wheelsVel(4);

    double omega_robot;
    double vx_robot;
    double vy_robot;
    double vx_gt;
    double vy_gt;
    double omega_gt;

    double vx;
    double vy;

    double loss;
    double gradient_step = 1e-5;

    int n_steps = 5000;

    std::vector<double> prev_pose(3);

    std::stringstream ss;

    while (ros::ok()) {
      
        gt_pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/robot/pose");
        wheels_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/wheel_states");

        if (prev_stamp != ros::Time(0)) {

            // computation of deltaTime in nsec
            deltaTime = wheels_state->header.stamp.sec - prev_stamp.sec + ((wheels_state->header.stamp.nsec - prev_stamp.nsec) / 1e9);

            // computation of deltaTicks
            std::transform((wheels_state->position).begin(), (wheels_state->position).end(), (prev_ticks).begin(), 
            std::back_inserter(deltaTicks), [](double l, double r) { return l - r;});

            // dividing deltaTicks for deltaTime
            transform(deltaTicks.begin(), deltaTicks.end(), deltaTicks.begin(), 
            [deltaTime](double &c){ return c/deltaTime;});

            // finding wheels velocity multiplying deltaTicks with multFactor
            wheelsVel = deltaTicks;
            transform(wheelsVel.begin(), wheelsVel.end(), wheelsVel.begin(), 
            [multFactor](double &c){ return c*multFactor;});

            // computing linear and angular speeds from given system info and wheels velocities
            omega_robot = (wheelsVel.at(1) - wheelsVel.at(2) + wheelsVel.at(3) - wheelsVel.at(0))*radius/(4*(l + w));
            vx_robot = (wheelsVel.at(0) + wheelsVel.at(1) + wheelsVel.at(2) + wheelsVel.at(3))*radius/4;


            tf2::Quaternion quaternion(gt_pose->pose.orientation.x, gt_pose->pose.orientation.y,gt_pose->pose.orientation.z,gt_pose->pose.orientation.w);
            
            double roll, pitch, theta;

            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, theta);

            vy_robot = (wheelsVel.at(1) + wheelsVel.at(2) - wheelsVel.at(3)- wheelsVel.at(0))*radius/4;
            vx = vx_robot * cos(theta) -  vy_robot * sin(theta);
            vy = vx_robot * sin(theta) + vy_robot * cos(theta);

            vx_gt = (gt_pose->pose.position.x - prev_pose.at(0)) / deltaTime;
            vy_gt = (gt_pose->pose.position.y - prev_pose.at(1)) / deltaTime;
            omega_gt = (theta - prev_pose.at(2)) / deltaTime;



            loss = pow(vx - vx_gt, 2) + pow(vy - vy_gt, 2) + pow(omega_robot - omega_gt, 2);

            ss << "Loss: " << loss << std::endl;
            ROS_INFO_STREAM(ss.str());

            double radius_derivative = (vx + vy + omega_robot) / radius - (vx_gt + vy_gt + omega_gt);
            radius += -radius_derivative * gradient_step;

            //update old variables
            prev_pose.at(0) = gt_pose->pose.position.x;
            prev_pose.at(1) = gt_pose->pose.position.y;
            prev_pose.at(2) = theta;

            prev_ticks = wheels_state->position;
            prev_stamp = wheels_state->header.stamp;

            ss << "Calibrated Radius: " << radius << std::endl;
            ROS_INFO_STREAM(ss.str());

            loop_rate.sleep();
        }

    }

    return 0;
}
