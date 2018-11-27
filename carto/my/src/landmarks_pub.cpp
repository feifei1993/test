#include "ros/ros.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "cartographer_ros_msgs/LandmarkEntry.h"
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>  
#include <std_msgs/Float64.h>

#define random(x) (rand()%x)

int num=0;
ros::Publisher landmarks_pub;
int k=0;
cartographer_ros_msgs::LandmarkList landmarks;

float xxx;
float yyy;

void Callback(nav_msgs::Odometry msg)
{
	geometry_msgs::TransformStamped t1;
	geometry_msgs::TransformStamped ti1;
    xxx = msg.pose.pose.position.x;
    yyy = msg.pose.pose.position.y;
	t1.transform.translation.x = msg.pose.pose.position.x;
	t1.transform.translation.y = msg.pose.pose.position.y;
	t1.transform.translation.z = msg.pose.pose.position.z;
	t1.transform.rotation.x = msg.pose.pose.orientation.x;
	t1.transform.rotation.y = msg.pose.pose.orientation.y;
	t1.transform.rotation.z = msg.pose.pose.orientation.z;
	t1.transform.rotation.w = msg.pose.pose.orientation.w;
	Eigen::Affine3d pose = tf2::transformToEigen(t1);
	Eigen::Affine3d pose_inverse = pose.inverse();
    ti1 = tf2::eigenToTransform(pose_inverse);

	landmarks.landmarks.clear();
	cartographer_ros_msgs::LandmarkEntry landmark1,landmark2;
   	landmark1.translation_weight = 1e5;
    landmark1.rotation_weight = 1e5;
    landmark1.id = "1000000";

	srand((int)time(0));
	float randa = (random(100)-50)*0.001;

    landmark1.tracking_from_landmark_transform.position.x = ti1.transform.translation.x;
    landmark1.tracking_from_landmark_transform.position.y = ti1.transform.translation.y;
    landmark1.tracking_from_landmark_transform.position.z = ti1.transform.translation.z;
    landmark1.tracking_from_landmark_transform.orientation.x = ti1.transform.rotation.x;
    landmark1.tracking_from_landmark_transform.orientation.y = ti1.transform.rotation.y;
    landmark1.tracking_from_landmark_transform.orientation.z = ti1.transform.rotation.z;
    landmark1.tracking_from_landmark_transform.orientation.w = ti1.transform.rotation.w;


	landmarks.header.stamp = ros::Time::now();
	landmarks.header.frame_id = "base_link";
	landmarks.landmarks.push_back(landmark1);
	num++;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "landmarks_pub");
    ros::NodeHandle n;
    landmarks_pub = n.advertise<cartographer_ros_msgs::LandmarkList>("landmark",1);
	ros::Subscriber ground_truth = n.subscribe("base_pose_ground_truth", 100, Callback);
	ros::Rate loop_rate(10);
	float x = 0.0;
	float y = 0.0;
    while(ros::ok())
	{
		if ( fabs(xxx-x)>1 || fabs(yyy-y)>1 )
		{
			landmarks_pub.publish(landmarks);
			x = xxx;
			y = yyy;
		}
		ros::spinOnce();
		loop_rate.sleep();	
	}
    return 0;
}
