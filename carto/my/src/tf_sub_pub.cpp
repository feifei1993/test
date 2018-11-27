#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"  
#include <iostream>      
#define PI 3.14159265
static ros::Publisher pub;
void laserCallback(sensor_msgs::LaserScan msg)
{
    
    msg.header.frame_id="laser";
    pub.publish(msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_listener");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::LaserScan>("scan_", 1000);
    ros::Subscriber sub = nh.subscribe("/scan", 1, laserCallback);
    ros::spin();
    return 0;
}
