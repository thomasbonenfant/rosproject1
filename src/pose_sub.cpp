#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include <sstream>

int main(int argc, char **argv) {

    ros::init(argc, argv, "pose_sub");

    ros::Subscriber sub;
    tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transform;

    geometry_msgs::PoseStampedConstPtr pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/robot/pose");

    std::stringstream ss;



    ss << std::setprecision(3);
    ss << "Received first Optitrack Pose!" << std::endl;
    ss << "Position" << std::endl;
    ss << pose->pose.position << std::endl;
    ss << "Orientation" << std::endl;
    ss << pose->pose.orientation << std::endl;

    ROS_INFO_STREAM(ss.str());

    transform.child_frame_id = "odom";
    transform.header.frame_id = "world";
    transform.header.stamp = ros::Time::now();

    transform.transform.translation.x = pose->pose.position.x;
    transform.transform.translation.y = pose->pose.position.y;
    transform.transform.translation.z = 0.0;

    transform.transform.rotation.w = pose->pose.orientation.w;
    transform.transform.rotation.x = pose->pose.orientation.x;
    transform.transform.rotation.y = pose->pose.orientation.y;
    transform.transform.rotation.z = pose->pose.orientation.z;

    br.sendTransform(transform);
    ros::spin();

    return 0;
}