#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include <sensor_msgs/LaserScan.h>//laserScan
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
//std header
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
//laser detect header
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <time.h>
#include "odometer_mecanum/robot_speed.h"
using namespace std;
using namespace cv;
nav_msgs::Odometry odoBuffer; //to 时间同步
 boost::mutex mMutexOdoBuffer;
sensor_msgs::LaserScan scanBuffer; //to 时间同步
 boost::mutex mMutexScanBuffer;
 int flag=0;
void chatterCallback(const nav_msgs::Odometry odoValue)//for odom
 //const odometer_mecanum::odometer &odoValue
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  //cout<<"get odo!"<<odoBuffer.size()<<endl;
  boost::mutex::scoped_lock lock(mMutexOdoBuffer);
  odoBuffer=odoValue;
}
void chatterCallback1(const sensor_msgs::LaserScan scanValue)//for laser
 //const odometer_mecanum::odometer &odoValue
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  //cout<<"get odo!"<<odoBuffer.size()<<endl;
  boost::mutex::scoped_lock lock(mMutexScanBuffer);
   scanBuffer=scanValue;
   flag=1;
}
const int thre=2;
int main(int argc, char **argv)
{
   ros::init(argc, argv, "calib");
   ros::NodeHandle n;
   ros::Subscriber odom_sub=n.subscribe< nav_msgs::Odometry>("odom",1000,chatterCallback);
   ros::Publisher roboSpeedPub= n.advertise<odometer_mecanum::robot_speed>("msg_robot_speed", 1);
   ros::Subscriber scan_sub=n.subscribe< sensor_msgs::LaserScan>("scan",1000,chatterCallback1);
   ros::Rate loop_rate(10);
   sleep(3);
   nav_msgs::Odometry odomdata;
   sensor_msgs::LaserScan scandata;
   double depth[10][500];
   int count=0;
   double x,y,w;
   double detx, dety, detw, detxc, detyc;
   double xpos,ypos,cita;
   odometer_mecanum ::robot_speed speedset;
   while(flag==0){ros::spinOnce();}
   
   while(ros::ok())
   {
     
     if(count==10)
     {
       double sum=0;
       int num=0;
       for(int i=1;i<10;i++)
	{
	  for(int j=100;j<400;j++)
	  {
	      if(depth[i][j]>1&&depth[i][j]<4&&depth[i-1][j]>1&&depth[i-1][j]<4&&depth[i-1][j]-depth[i][j]>(10-thre)/100.0&&depth[i-1][j]-depth[i][j]<(10+thre)/100.0)
	      {
		cout<<depth[i-1][j]-depth[i][j]<<endl;
		sum+=depth[i-1][j]-depth[i][j];
		num++;
	      }
	  }
	}
	double a=0.1/(sum/num);
	cout<<a<<endl;
	return 0;
	
     }
     //cout<<"ddd"<<endl;
     {
	boost::mutex::scoped_lock lock(mMutexOdoBuffer);
	odomdata=odoBuffer;
     } 
     {
	boost::mutex::scoped_lock lock(mMutexScanBuffer);
	scandata=scanBuffer;
     }
     if(count==0)
     {
        x=odomdata.pose.pose.position.x ;
	y=odomdata.pose.pose.position.y ;
	w=odomdata.pose.pose.position.z ;
	xpos=odomdata.pose.pose.position.x ;
	ypos=odomdata.pose.pose.position.y ;
	cita=odomdata.pose.pose.position.z ;
	
     }
     else
     {
       xpos=odomdata.pose.pose.position.x ;
	ypos=odomdata.pose.pose.position.y ;
	cita=odomdata.pose.pose.position.z ;  
     }
     
	
	detx = xpos - x;
	dety = ypos - y;
	detw = cita - w;
	detxc = detx*cos(-w) - dety*sin(-w);
	detyc = detx*sin(-w) + dety*cos(-w);
	//cout<<detxc<<" "<<detyc<<endl;
	/*
	 for(int k=0;k<500;k++)
         {
	  double c=scandata.ranges[k];
	  double angle=k*scandata.angle_increment;
	  angle=angle-3.1415926/4;
	  angle=3.1415926/2-angle;
	  double xx=c*cos(angle);
	  double yy=sqrt(c*c-xx*xx);
	  depth[count][k]=yy;
	  if(k%50==0)
	  {
	    cout<<yy<<" ";
	  }
	  
	  
	}
	cout<<endl;
	*/
     if(fabs(detxc-count*0.1)<0.01)
     {
	 for(int k=0;k<500;k++)
         {
	  double c=scandata.ranges[k];
	  double angle=k*scandata.angle_increment;
	  angle=angle-3.1415926/4;
	  angle=3.1415926/2-angle;
	  double xx=c*cos(angle);
	  double yy=sqrt(c*c-xx*xx);
	  depth[count][k]=yy;
	  
	}
	count++;
     }
     speedset.vx=5;
     roboSpeedPub.publish(speedset);
    ros::spinOnce();
     
     
   }
   
   
  
  
   
   
   
   
   
}
