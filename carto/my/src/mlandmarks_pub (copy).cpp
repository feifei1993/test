#include "ros/ros.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "cartographer_ros_msgs/LandmarkEntry.h"
//#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <time.h> 
#include <vector>
#include <sstream>
//#include "tf/transform_datatypes.h"
#define random(x) (rand()%x)
#define max 100

int num=0;
ros::Publisher landmarks_pub;
std::vector<geometry_msgs::TransformStamped> vg;

void Callback(const nav_msgs::Odometry msg)
{
	geometry_msgs::TransformStamped t;
	geometry_msgs::TransformStamped ti;
	cartographer_ros_msgs::LandmarkList landmarks;
	t.transform.translation.x = msg.pose.pose.position.x;
	t.transform.translation.y = msg.pose.pose.position.y;
	t.transform.translation.z = msg.pose.pose.position.z;
	t.transform.rotation.x = msg.pose.pose.orientation.x;
	t.transform.rotation.y = msg.pose.pose.orientation.y;
	t.transform.rotation.z = msg.pose.pose.orientation.z;
	t.transform.rotation.w = msg.pose.pose.orientation.w;
	Eigen::Affine3d te = tf2::transformToEigen(t);
    ti = tf2::eigenToTransform(te.inverse());
	srand((int)time(0));
	if(num>10)
	{
		for(int i=0;i<vg.size();i++)
		{
			cartographer_ros_msgs::LandmarkEntry landmark;
			geometry_msgs::TransformStamped g = vg[i];
			Eigen::Affine3d ge = tf2::transformToEigen(g);
			Eigen::Affine3d ge2;
			tf2::doTransform (ge, ge2, ti);
			g = tf2::eigenToTransform(ge2);

			float randa = (random(100)-50)*0.0001;
			landmark.translation_weight = 1e5;
            landmark.rotation_weight = 1e5;
			std::stringstream ss;
			ss << i;
            landmark.id = ss.str();
			landmark.tracking_from_landmark_transform.position.x = g.transform.translation.x + randa;
    		landmark.tracking_from_landmark_transform.position.y = g.transform.translation.y + randa;
    		landmark.tracking_from_landmark_transform.position.z = g.transform.translation.z;
    		landmark.tracking_from_landmark_transform.orientation.x = g.transform.rotation.x;
    		landmark.tracking_from_landmark_transform.orientation.y = g.transform.rotation.y;
    		landmark.tracking_from_landmark_transform.orientation.z = g.transform.rotation.z;
    		landmark.tracking_from_landmark_transform.orientation.w = g.transform.rotation.w;
			landmarks.landmarks.push_back(landmark);
		}
	}
	landmarks.header.stamp = ros::Time::now();
	landmarks.header.frame_id = "base_link";
    landmarks_pub.publish(landmarks);
	num++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landmarks_pub");
    ros::NodeHandle n;
	int m=10;
	int l=-3;
	int r=6;
	int u=5;
	int d=-2;
	srand((int)time(0));
	for(int j=0;j<m;j++)//生成m个固定landmark
	{
		//产生随机数
		float w = (random(((r-l)*100)+l*100))*0.01;
		float h = (random(((u-d)*100)+d*100))*0.01;
		//
		geometry_msgs::TransformStamped g;
		g.transform.translation.x = w;
		g.transform.translation.y = h;
		g.transform.translation.z = 0.0;
		g.transform.rotation.x = 0.0;
		g.transform.rotation.y = 0.0;
		g.transform.rotation.z = 0.0;
		g.transform.rotation.w = 1.0;
		vg.push_back(g);
		//std::cout<<w<<" "<<h<<std::endl;
	}
    landmarks_pub = n.advertise<cartographer_ros_msgs::LandmarkList>("landmark",1);
	ros::Subscriber ground_truth = n.subscribe("base_pose_ground_truth", 100, Callback);
	ros::spin();
    return 0;
}
