#include "ros/ros.h"
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/transform_datatypes.h"

int main(int argc, char **argv)
{
		geometry_msgs::TransformStamped ti1;
		geometry_msgs::TransformStamped ti2;
		geometry_msgs::TransformStamped t1;
		geometry_msgs::TransformStamped t2;
		
		t1.transform.translation.x=2.0;
		t1.transform.translation.y=2.0;
		t1.transform.translation.z=0.0;
		t1.transform.rotation.x=0.0;
		t1.transform.rotation.y=0.0;
		t1.transform.rotation.z=0.0;
		t1.transform.rotation.w=1.0;

		t2.transform.translation.x=2.0;
		t2.transform.translation.y=2.0;
		t2.transform.translation.z=0.0;
		t2.transform.rotation.x=0.0;
		t2.transform.rotation.y=0.0;
		t2.transform.rotation.z=0.0;
		t2.transform.rotation.w=1.0;

		Eigen::Affine3d pose = tf2::transformToEigen(t1);//转换到齐次矩阵[r,t;0,1]
		//std::cout<<pose(0,3)<<" "<<pose(1,3)<<" "<<pose(2,3)<<" "<<pose(3,3)<<std::endl;
		Eigen::Affine3d pose_inverse = pose.inverse();
        ti1 = tf2::eigenToTransform(pose_inverse);

		tf::Quaternion quat;
	    tf::quaternionMsgToTF(ti1.transform.rotation, quat);
		double roll, pitch, yaw;
    	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		//std::cout<<tiq.transform.translation.x<<" "<<ti1.transform.translation.y<<" "<<yaw<<std::endl;

		Eigen::Affine3d t_in= tf2::transformToEigen(t2);

		Eigen::Affine3d t_out;
        tf2::doTransform (t_in, t_out, ti1);
		ti2 = tf2::eigenToTransform(t_out);
		std::cout<<ti2.transform.translation.x<<" "<<ti2.transform.translation.y<<" "<<std::endl;
		return 0;
}
