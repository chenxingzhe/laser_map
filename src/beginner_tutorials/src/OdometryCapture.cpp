/************************************************************************/
/* Copyright (c) CSC-RL, Zhejiang University							*/
/* Team:		ZJUPanda@Home											*/
/* HomePage:															*/
/************************************************************************/
/* File:	OdometryCalculation.cpp										*/
/* Brief:	C++ Implementation: Odometry								*/
/* Func:	provide the odometry information as control 				*/
/* Author:	cliffyin, 2012, 04											*/
/* Refer:	Kinematics of omni-direction								*/
/* E-mail:	cliffyin@zju.edu.cn		cliffyin007@gmail.com				*/
/************************************************************************/

#include "OdometryCapture.h"
//#include "mmsystem.h"
//#pragma comment(lib,"winmm")
using namespace std;
#include <fstream>
//#include <process.h>
#include <ctime>
#include <iomanip>
#include "OdometryCalculation.h"
#include "MecanumModule.h"
#include "NRF_Common/NRF_Utils.h"
#include <boost/thread/mutex.hpp>
#include <tf/transform_broadcaster.h>
extern ros::Publisher m_odom;
extern ros::Publisher s_laser;
COdometerCapture::COdometerCapture()
{
	m_initial = false;					// 是否已经启动
	m_hThread = NULL;					// 线程句柄
	//m_threadID = NULL;					// 线程ID
	m_hEventOdoCalTimer = NULL;			// 里程计合成触发事件
	m_hEventOdoPubTimer = NULL;			// 里程计发布触发事件
	m_hEventSpeedTimer = NULL;			// 下发速度触发事件
	m_hEventSpeedCheckTimer = NULL;		// 下发速度检查触发事件

	m_lost_time = 0.0;					// 下发速度丢失时间
}

COdometerCapture::~COdometerCapture()
{
	destroy();
}

void COdometerCapture::initial()
{
	// 开启控制线程：处理上面设置的与里程计和速度相关的时间
    m_initial = true;
	int TIMERDELAY = 10;
	timer_odo_call = timeSetEvent(1000,TIMERDELAY,NULL,NULL,false);
	if(timer_odo_call==NULL)
    {
        std::cout<<"Odo Cal: Create Event Error!"<<std::endl;
        return;
    }
    else
    {
        std::cout << "Odo Cal Timer OK!" << std::endl;
    }
	m_hEventOdoCalTimer=timer_odo_call->timerID;

	// 注册里程计发布事件
	TIMERDELAY = 47;
	timer_odo_pub = timeSetEvent(1000,TIMERDELAY,NULL,NULL,false);
	if(timer_odo_pub==NULL)
    {
        std::cout<<"Odo pub: Create Event Error!"<<std::endl;
        return;
    }
    else
    {
        std::cout << "Odo Pub Timer OK!" << std::endl;
    }
    m_hEventOdoPubTimer = timer_odo_pub->timerID;

	// 注册下发速度事件
	TIMERDELAY = 47;
	timer_speed = timeSetEvent(1000,TIMERDELAY,NULL,NULL,false);
	if(timer_speed==NULL)
    {
        std::cout<<"Odo speed: Create Event Error!"<<std::endl;
        return;
    }
    else
    {
        std::cout << "Odo speed Timer OK!" << std::endl;
    }
    m_hEventSpeedTimer = timer_speed->timerID;
	// 注册下发速度检查事件
	TIMERDELAY = 250;
	timer_speedcheck = timeSetEvent(1000,TIMERDELAY,NULL,NULL,false);
	if(timer_speedcheck==NULL)
    {
        std::cout<<"Odo speed check: Create Event Error!"<<std::endl;
        return;
    }
    else
    {
        std::cout << "Odo speed check Timer OK!" << std::endl;
    }
    m_hEventSpeedCheckTimer = timer_speedcheck->timerID;
    int res = pthread_create(&m_hThread,NULL,ControlThread,this);
    //boost::thread ControlThread(boost::bind())
    if(res !=0){
        std::cout << "thread create error" << std::endl;
        return;
    }
	return;
}

void COdometerCapture::StartOdometryCapture()
{
	initial();
	m_check_timer.start();
	return;
}

/// Set current speed
void COdometerCapture::SetCurSpeed(const geometry_msgs::Twist& robotSpeed,  bool checkSet)
{
	m_mutex_speed.Lock();

	if (! checkSet) {
		m_mutex_lost.Lock();
		m_lost_time = 0.0;
		m_mutex_lost.Unlock();
	}
    m_robotSpeed.linear.x= robotSpeed.linear.x;
	m_robotSpeed.linear.y= robotSpeed.linear.y;
	m_robotSpeed.angular.z = robotSpeed.angular.z;
	m_mutex_speed.Unlock();
	return ;
}

void COdometerCapture::SetCurLaser(const sensor_msgs::LaserScan& LaserData)
{
   m_laser_mutex.Lock();
   m_LaserData = LaserData;
   m_laser_mutex.Unlock();
}
/// Get current speed
void COdometerCapture::GetCurSpeed(geometry_msgs::Twist& robotSpeed)
{
	m_mutex_speed.Lock();
	robotSpeed.linear.x = m_robotSpeed.linear.x;
	robotSpeed.linear.y = m_robotSpeed.linear.y;
	robotSpeed.angular.z   = m_robotSpeed.angular.z;
	m_mutex_speed.Unlock();

	return ;
}

/// Speed check
void COdometerCapture::SpeedCheck()
{
	m_check_timer.stop();
	m_mutex_lost.Lock();
	m_lost_time += m_check_timer.getMsecTime();
	m_mutex_lost.Unlock();
	m_check_timer.start();

	return ;
}

/// Safe speed
void COdometerCapture::DoSafeSpeed()
{
	double cur_lost_time = 0.0;
	m_mutex_lost.Lock();
	cur_lost_time = m_lost_time;
	m_mutex_lost.Unlock();

	if (cur_lost_time > 1000.0) {	// 速度丢失了1s，降速
		geometry_msgs::Twist cur_robotSpeed;
		GetCurSpeed(cur_robotSpeed);

		double vx_step = 0.25;
		double cur_vx = cur_robotSpeed.linear.x;
		if (0 != cur_vx) {
			if (cur_vx > 0) {
				cur_vx -= vx_step;
			} else {
				cur_vx += vx_step;
			}
		}

		double vy_step = 0.25;
		double cur_vy = cur_robotSpeed.linear.y;
		if (0 != cur_vy) {
			if (cur_vy > 0) {
				cur_vy -= vy_step;
			} else {
				cur_vy += vy_step;
			}
		}

		double w_step = 0.25;
		double cur_w = cur_robotSpeed.angular.z;
		if (0 != cur_w) {
			if (cur_w > 0) {
				cur_w -= w_step;
			} else {
				cur_w += w_step;
			}
		}

		//cur_robotSpeed.set_vx(cur_vx);
		//cur_robotSpeed.set_vy(cur_vy);
		//cur_robotSpeed.set_w(cur_w);
		cur_robotSpeed.linear.x = cur_vx;
		cur_robotSpeed.linear.y = cur_vy;
		cur_robotSpeed.angular.z = cur_w;

		SetCurSpeed(cur_robotSpeed, true);
	}

	return ;
}

void COdometerCapture::GetCurOdometry(nav_msgs::Odometry& odometerData)
{
	//odometerData.Clear();

	double odo_x = 0.0;
	double odo_y = 0.0;
	double odo_angle = 0.0;
	ODOCAL->getCurOdo(odo_x,odo_y,odo_angle);

	double odo_w1 = 0.0;
	double odo_w2 = 0.0;
	double odo_w3 = 0.0;
	double odo_w4 = 0.0;
	ODOCAL->getAllWheels(odo_w1,odo_w2,odo_w3,odo_w4);
    odometerData.pose.pose.position.x = odo_x;
	odometerData.pose.pose.position.y = odo_y;
	odometerData.pose.pose.position.z = 0;
	odometerData.pose.pose.orientation.z = odo_angle;
	return ;
}

void COdometerCapture::GetCurLaser(sensor_msgs::LaserScan& laserData)
{
    m_laser_mutex.Lock();
    laserData = m_LaserData;
    m_laser_mutex.Unlock();
}

void COdometerCapture::closeAll()
{
	if (m_hEventOdoCalTimer) {
		timeKillEvent(timer_odo_call);
	}

	if (m_hEventOdoPubTimer) {
		timeKillEvent(timer_odo_pub);
	}

	if (m_hEventSpeedTimer) {
		timeKillEvent(timer_speed);
	}

	if (m_hEventSpeedCheckTimer) {
		//CloseHandle(m_hEventSpeedCheckTimer);
		timeKillEvent(timer_speedcheck);
	}

	m_hEventOdoCalTimer = NULL;
	m_hEventOdoPubTimer	= NULL;
	m_hEventSpeedTimer = NULL;
	m_hEventSpeedCheckTimer = NULL;

	return ;
}

void COdometerCapture::destroy()
{
	if (m_initial) {
		//SetEvent(m_hEventOdoCalTimer);
		//WaitForSingleObject(m_hThread, INFINITE);

		pthread_cancel(m_hThread);
		closeAll();
		m_initial = false;
	}

	return ;
}

void time_check_print(string head)
{
	static NRF_CTimer cur_timer;
	cur_timer.stop();
	cout << head << " cur_eclipse_time : " << cur_timer.getMsecTime() / 1000.0f << " ms" << endl;
	cur_timer.start();

	return ;
}

//#define  ODOM_AVERAGE_SIZE 3

void* ControlThread(void *p)
{
	COdometerCapture *pComm = (COdometerCapture*)p;

	/// Handling events
	const int EVENT_NUM = 4;
	HANDLE hWait[EVENT_NUM];
	hWait[0] = pComm->m_hEventOdoCalTimer;
	hWait[1] = pComm->m_hEventOdoPubTimer;
	hWait[2] = pComm->m_hEventSpeedTimer;
	hWait[3] = pComm->m_hEventSpeedCheckTimer;

	/// Real Send Speed
//	double real_vx = 0.0;
//	double real_vy = 0.0;
//	double real_w = 0.0;
	// allowance change value for a single cycle
//	double allow_vx_change_per_cycle = 2.5;		// cm/s
//	double allow_vx_decrease_per_cycle = 5.75;    //cm/s
//	double allow_vy_change_per_cycle = 2.5;		// cm/s
//	double allow_w_increase_per_cycle = 4.5;		// deg/s
//	double allow_w_decrease_per_cycle = 4.5;		// deg/s
//
//	extern double g_closest_obstacle_x;  //urg看到的最近障碍物点 x,y坐标
//	extern double g_closest_obstacle_y;  //urg看到的最近障碍物点
//	extern double g_max_acc_inc_x;  //最大加速度
//	extern double g_max_acc_dec_x;  //最减加速度
//	extern double g_max_acc_w;
//	extern double g_max_vel_x; //最大速度
//	extern double g_max_vel_w; //最大角速度
	extern boost::mutex g_mutex_obstacle; //

//	double cur_ac_vx,cur_ac_vy,cur_ac_vw;
//	double old_x = 0,old_y = 0,old_w = 0;
	boost::posix_time::ptime prev_odom_time;
	boost::mutex vel_mutex;


	/// Handling loop
	fd_set fdset;
	//uint64_t exp;
	struct timeval timeout;
	timeout.tv_sec=5;
	timeout.tv_usec=0;
	while (pComm->m_initial)
	{
        FD_ZERO(&fdset);
        int maxfp=0;
	    for(int k=0;k<EVENT_NUM;k++)
	    {
	        FD_SET(hWait[k],&fdset);
	        if(maxfp<hWait[k])
               {
                    maxfp=hWait[k];
               }
	    }
	    int dRes = select(maxfp+1,&fdset,NULL,NULL,&timeout);
		switch (dRes)
		{
    //*********************************//
    //        修改case后面的值                     //
    //********************************//
        case -1 :
                cout<<"error of select"<<endl;
        case 0 :
                cout<<"No Data within five seconds"<<endl;
		default :
		    if(FD_ISSET(hWait[0],&fdset))						// 里程计合成
			{
				pComm->m_mutex_cap.Lock();
				ODOCAL->onTimerCal2(); /// Timer on calculation - using position
				pComm->m_mutex_cap.Unlock();
			}
            if(FD_ISSET(hWait[1],&fdset))						// 里程计发布
			{
				pComm->m_mutex_cap.Lock();
                nav_msgs::Odometry odometerData;
                pComm->GetCurOdometry(odometerData);
                odometerData.header.stamp = ros::Time::now();
                odometerData.header.frame_id = "odom";
                odometerData.child_frame_id = "base_link";
                static tf::TransformBroadcaster odom_broadcaster;
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometerData.pose.pose.orientation.z);
                ///first, we'll publish odom the transform over tf
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = ros::Time::now();
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link";

                odom_trans.transform.translation.x = odometerData.pose.pose.position.x;
                odom_trans.transform.translation.y = odometerData.pose.pose.position.y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quat;
                ///send the transform
                odom_broadcaster.sendTransform(odom_trans);
                ///publish laser transform over tf
                ////sensor_msgs::LaserScan LaserData;
               // pComm->GetCurLaser(LaserData);
                static tf::TransformBroadcaster laser_broadcaster;
                tf::Transform laser_transform;
                laser_transform.setOrigin( tf::Vector3(0.3, 0, 0.05) );
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                laser_transform.setRotation(q);
                laser_broadcaster.sendTransform(tf::StampedTransform(laser_transform, ros::Time::now(), "base_link", "laser"));
//                geometry_msgs::Quaternion laser_quat = tf::createQuaternionMsgFromYaw(0);
//                geometry_msgs::TransformStamped laser_trans;
//                LaserData.header.stamp = ros::Time::now();
//                laser_trans.transform.translation.x = 0.3;
//                laser_trans.transform.translation.y = 0;
//                laser_trans.transform.translation.z = 0.05;
//                laser_trans.transform.rotation = laser_quat;
                //s_laser.publish(LaserData);
                m_odom.publish(odometerData);
				pComm->m_mutex_cap.Unlock();
			}
            if(FD_ISSET(hWait[2],&fdset))                    // 速度下发
			{
				geometry_msgs::Twist sendSpeed;
				pComm->GetCurSpeed(sendSpeed);
				double send_vx = sendSpeed.linear.x;
				double send_vy = sendSpeed.linear.y;
				double send_w = sendSpeed.angular.z;
				//限速
				send_vx = Utils::clip(send_vx,-1000.0,1000.0);
				send_vy = Utils::clip(send_vy,-1000.0,1000.0);
				send_w = Utils::clip(send_w,-50.0,50.0);
				MECANUM->SendVelocities(send_vx/100.0, send_vy/100.0, send_w*3.1415926/180.0);
				pComm->DoSafeSpeed();
			}
            if(FD_ISSET(hWait[3],&fdset))                    // 速度下发检查
			{
				pComm->SpeedCheck();
			}
			break;
		}
	}
	return 0;
}
