#include "ros/ros.h"
#include <iostream>
#include <fstream>   
#include <geometry_msgs/Twist.h>    
#include <geometry_msgs/TwistStamped.h> 
#include <geometry_msgs/Quaternion.h>    
    
#include <nav_msgs/Odometry.h>   
#include <tf/LinearMath/Matrix3x3.h>
      
#define PI 3.14159265

std::ofstream myfile("/home/ares/ros_log/amcl/stage_pose.txt",std::ios::out);
void odoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;    
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));    
    double yaw, pitch, roll;    
    mat.getEulerYPR(yaw, pitch, roll);  
    //myfile<<msg->header.stamp<<std::endl;时间戳
    myfile<<msg->pose.pose.position.x<<" "<<msg->pose.pose.position.y<<" "<<yaw*180/PI<<std::endl;
    //myfile<<msg->twist.twist.linear.x<<" "<<msg->twist.twist.angular.z<<std::endl;速度
}
int main(int argc, char **argv)
{
    if(!myfile)
    {
        std::cout<<"open file failure"<<std::endl;
    }
    ros::init(argc, argv, "laser_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/base_pose_ground_truth", 1, odoCallback);
    ros::spin();
    myfile.close();
    return 0;
}
