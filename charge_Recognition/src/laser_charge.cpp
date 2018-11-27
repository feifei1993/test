#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h" 
#include "geometry_msgs/Pose2D.h" 
#include <iostream>
#include <vector>      
#define PI 3.14159265
static ros::Publisher charge_pose_pub;
class line
{
	public:
		std::vector<float> x;
		std::vector<float> y;
		float d;
};

std::vector<line> linesNew;

void line_fit(std::vector<float> x, std::vector<float> y, float &k, float &b)//直线拟合
{
	float A = 0.0;
	float B = 0.0;
	float C = 0.0;
	float D = 0.0;
	float temp = 0;
	for (int i = 0; i < x.size(); i++)
	{
		A += x[i] * x[i];
		B += x[i];
		C += x[i] * y[i];
		D += y[i];
	}
	temp = (x.size()*A - B*B);
	if (temp!=0)	
	{
		k=((x.size()*C - B*D)/temp);
		b=((A*D - B*C)/temp);
	}
	else
	{
		k=0;
		b=1;
	}

}
void polar2Cart(std::vector<float> &laser,std::vector<float> &x,std::vector<float> &y,std::vector<float> &gate,float gatemul)//极坐标转直角坐标
{
	std::vector<float> lasernew;
	int ls=laser.size();
	for(int i=0;i<ls;i++)
	{
		if(!isinf(laser[i]))//delet inf
		{
			lasernew.push_back(laser[i]);
			float rad=i*4.0/689-2.0+PI/2.0;
            //std::cout<<i <<" "<<rad <<" ";
			float xx=laser[i] * cos(rad);
			float yy=laser[i] * sin(rad);
			x.push_back(xx);
			y.push_back(yy);
			float g=gatemul*sqrt( xx*xx + yy*yy )*sin(PI/180);//动态阈值
			gate.push_back(g);
		}
	}
	laser=lasernew;
}
std::vector<line> regionSegement(std::vector<float> &x,std::vector<float> &y,std::vector<float> &gate,int linesize)//将原始数据分割
{
	std::vector<line> lines;
	float dist;
	int gs=gate.size()-1;
	int id=0;
	for (int i=0;i<gs;i++)
	{
		dist=sqrt( (y[i]-y[i+1])*(y[i]-y[i+1]) + (x[i]-x[i+1])*(x[i]-x[i+1]) );
		if(dist>gate[i])
		{
			if(abs(i-id)>linesize)
			{
				line l;
				std::vector<float> xx,yy;
				//              std::vector<float> xx(x.begin()+id,x.begin()+i);
				//			    std::vector<float> yy(y.begin()+id,y.begin()+i);
				for(int j=id;j<=i;j++)
				{
					xx.push_back(x[j]);
					yy.push_back(y[j]);
				}
				l.x=xx;
				l.y=yy;
				lines.push_back(l);
			}
			id=i+1;
		}
	}
	return lines;
}
void linesBreak(line l,float esp)//道格拉斯迭代分割
{
	std::vector<float> x=l.x;
	std::vector<float> y=l.y;
	int n=x.size();
	int id=0;
	float x1=x[0];
	float y1=y[0];
	float x2=x[n-1];
	float y2=y[n-1];
	float dis=sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
	float lcos=(x2-x1)/dis;
	float lsin=-(y2-y1)/dis;
	float maxdis=0;
	for(int i=0;i<n;i++)
	{
		float mdis=abs( (x[i]-x1)*lsin + (y[i]-y1)*lcos );
		if (mdis>maxdis)
		{
			maxdis=mdis;
			id=i;
		}
	}
	if (maxdis<esp || n<4)//如果小于阈值停止分割
	{
		l.d=dis;
		linesNew.push_back(l);
	}
	else//如果大于阈值继续分割
	{
		line ll;
		std::vector<float> xx,yy;
		for(int j=0;j<id;j++)
		{
			xx.push_back(x[j]);
			yy.push_back(y[j]);
		}
		ll.x=xx;
		ll.y=yy;
		linesBreak(ll,esp);

		xx.clear();
		yy.clear();
		for(int j=id;j<n;j++)
		{
			xx.push_back(x[j]);
			yy.push_back(y[j]);
		}
		ll.x=xx;
		ll.y=yy;
		linesBreak(ll,esp);
	}
} 
void lineIntersect(line &l1,line &l2,float &cx,float &cy,float &ang)
{
	float k1,k2,b1,b2;
	line_fit(l1.x,l1.y,k1,b1);//拟合直线
	line_fit(l2.x,l2.y,k2,b2);//拟合直线
	cx=(b2-b1)/(k1-k2);
	cy=k1*cx+b1;
	ang=abs( (atan(k1)-atan(k2)) * 180/PI );
	if (ang<90)
	{
		ang=(180-ang);
	}
}
line lineCut(line l)
{
	line lNew;
	for(int i=5;i<l.x.size()-5;i++)//遍历删除首位各两个点
	{
		lNew.x.push_back(l.x[i]);
		lNew.y.push_back(l.y[i]);
	}
	return lNew;
}
float pGet(std::vector<float> x,std::vector<float> y,float k,float b)
{
	float sum=0;
	for(int i=0;i<x.size();i++)//遍历删除首位各两个点
	{
		sum=sum+(y[i]-(k*x[i]+b));
	}
	return sum/x.size();
}
std::vector<float> charge_Recognition(std::vector<float> laser)
//void charge_Recognition(std::vector<float> laser)
{
	std::vector<float> pose;
    pose.push_back(-1);
    pose.push_back(-1);
    pose.push_back(-1);
	int linesize=10;
	float eps=0.05;
	float gate_mul=2;
	float l1d=0.25;
	float ldet=0.05;
	float k,b,ang,cx,cy,lineP,P;
	std::vector<float> x,y,gate;
	std::vector<line> lines;
	polar2Cart(laser,x,y,gate,gate_mul);//极坐标转直角坐标系
	lines=regionSegement(x,y,gate,linesize);//区域分割
	for(int i=0;i<lines.size();i++)
	{
		line l=lines[i];
		linesBreak(l,eps);//道格拉斯分割
		if(linesNew.size()==1)
		{
			line l1=linesNew[0];
        
			if (abs(l1.d-l1d)<ldet && l1.x.size()>12)//距离满足条件，点个数满足条件
			{
                //std::cout<<l1.x[0]<<" "<<l1.x[l1.x.size()-1]<<" "<<l1.y[0]<<std::endl;
                if( fabs(l1.x[0])<1 && fabs(l1.x[l1.x.size()-1])<1 && fabs(l1.y[0])>0.3)
					{
					l1=lineCut(l1);//删除首末各四个点
					line_fit(l1.x, l1.y, k, b);//拟合直线得k,b
					float sum=0;
					for( int j=0;j<l1.x.size();j++)
					{
						sum=sum+l1.x[j];
					}
					cx=sum/l1.x.size();//求x均值
					cy=k*cx+b;//代入拟合直线的k,b求y
					ang=atan(k)*180/PI;//由k求夹角
					//lineP=l1.d/l1d; //距离概率
					//P=pGet(l1.x,l1.y,k,b);//离散程度概率
					pose[0]=cx;
					pose[1]=cy;
					pose[2]=ang;
					//std::cout<<cx<<" "<<cy<<" "<<ang<<" "<<std::endl;
					}
			}
		}
		linesNew.clear();
	}
	return pose;	
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	std::vector<float> laser = msg->ranges;
	std::vector<float> pose;
	pose = charge_Recognition(laser);
    geometry_msgs::Pose2D c_pose;
    c_pose.x=pose[0];
	c_pose.y=pose[1];
    c_pose.theta=pose[2];
    //std::cout<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<std::endl;
    charge_pose_pub.publish(c_pose);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_listener");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/scan", 1, laserCallback);
	charge_pose_pub=nh.advertise<geometry_msgs::Pose2D>("charge_pose",100);
	ros::spin();
	return 0;
}
