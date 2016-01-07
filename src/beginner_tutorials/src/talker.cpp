/*
* Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*   * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
//ros header
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include <sensor_msgs/LaserScan.h>//laserScan
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
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
#include "lsd_opencv.hpp"
using namespace std;
using namespace cv;
//control bar
int threshold_value1 = 0;
int threshold_value2 = 15;
int threshold_value3 = 150;
int threshold_value4 = 100;
int threshold_value5 = 512;
char* trackbar_value1 = "H_low Value";
char* trackbar_value2 = "H_high Value";
char* trackbar_value3 = "angle_low Value";
char* trackbar_value4 = "angle_high Value";
char* trackbar_value5 = "HIGH";

Mat cameraMatrix, distMatrix, warp3D, warp3DInv;
//

double cameraPara[9] = {367.871016672033,	0	,315.825462320025,
                        0	,368.766777853911,	278.228082869275,
                        0	,0	,1};
double distorPara[4] = { 0.113921169702272,	-0.157613953720350	,0,	0 };//-0.140357318324816
//double distorPara[4] = { 0,0,0,0 };
//double para[5] = {  0.419985441915021	,-0.906982992790618	,0.0310764999626740	,-0.00532264202160894	,172.582853473145};
double para[5] = {  0.470662533156081	,-0.882261736383587	,-0.00779733965388201	,-0.00549635250342565	,648.344687834478};


/*
double cameraPara[9] = {390.906661634462	,0	,488.336327681434,
0	,409.411259693664	,114.058683071483,
0	,0	,1};
double distorPara[4] = {-0.164600797057878	,0.0185621968642294	,0	,0};//-0.140357318324816
//double distorPara[4] = { 0,0,0,0 };
//double para[5] = {  0.419985441915021	,-0.906982992790618	,0.0310764999626740	,-0.00532264202160894	,172.582853473145};
double para[5] = { 0.539266699580578	,-0.805317570623778	,-0.107877011602426	,-0.221399158841710	,1043.82037293902};
*/
double a[640], b[640], c[640];
void Threshold_Demo(int, void*)
{}
string num2str(int i){
    stringstream s;
    s << i;
    return s.str();
}
int my_cmp(double p1, double  p2)
{
    return p1> p2;
}
void updatePara(){

    Mat(3, 3, CV_64FC1, cameraPara).copyTo(cameraMatrix);
    Mat(4, 1, CV_64FC1, distorPara).copyTo(distMatrix);//ÉîžŽÖÆ

    double q0 = para[0], q1 = para[1], q2 = para[2], q3 = para[3], h = para[4];
    double R[9] = {
        2 * q0*q0 + 2 * q1*q1 - 1, 2 * q1*q2 - 2 * q0*q3, 2 * q1*q3 + 2 * q0*q2,
        2 * q1*q2 + 2 * q0*q3, 2 * q0*q0 + 2 * q2*q2 - 1, 2 * q2*q3 - 2 * q0*q1,
        2 * q1*q3 - 2 * q0*q2, 2 * q2*q3 + 2 * q0*q1, 2 * q0*q0 + 2 * q3*q3 - 1
    };

    Mat R_M(3, 3, CV_64F, (void *)R);
    warp3D = R_M*cameraMatrix.inv();
    warp3D.row(0) *= -h;
    warp3D.row(1) *= -h;

    warp3DInv = warp3D.inv();
    cout << cameraMatrix << endl << distMatrix << endl;
    //warp3DBias = warp3D;
    //warp3DBias.row(0) += 500;
    //warp3DBias.row(1) -= 1000;
}
void getPoint3D(double u, double v, double &x, double &y){
    double axisInImage[3] = { u, v, 1 };
    cv::Mat axisImage = cv::Mat(3, 1, CV_64FC1, axisInImage);
    cv::Mat_<double> divder = warp3D.row(2)*axisImage; double divderD = divder(0, 0);
    cv::Mat_<double> x_ = warp3D.row(0)*axisImage / divderD;
    cv::Mat_<double> y_ = warp3D.row(1)*axisImage / divderD;
    x = x_(0, 0);
    y = y_(0, 0);
}
void wrongPointDetect(double p[])
{

    for (int i = 1; i < 640-1; i++)
    {
        if (fabs(p[i - 1] - p[i + 1])<10 && fabs(p[i] - p[i - 1])>50)
        p[i] = (p[i + 1] + p[i - 1]) / 2;

    }
}

 std::list<nav_msgs::Odometry> odoBuffer; //to Ê±ŒäÍ¬²œ
 boost::mutex mMutexOdoBuffer;
 void chatterCallback(const nav_msgs::Odometry odoValue)
 //const odometer_mecanum::odometer &odoValue
 {
     //ROS_INFO("I heard: [%s]", msg->data.c_str());
     //cout<<"get odo!"<<odoBuffer.size()<<endl;
     boost::mutex::scoped_lock lock(mMutexOdoBuffer);
     odoBuffer.push_back(odoValue);
 }
 /**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
 void myRange(Mat input,Mat &output,int value1,int value2,int thre1,int thre2)
 {
     CvSize s=input.size();
     int height=s.height;
     int width=s.width;
     output=Mat(height,width,CV_8UC1);
     int dp[480]; 
     bool num[480];
     for(int j=0;j<width;j++)
     {

         for(int i=0;i<height;i++)
         {
             num[i]=0;
             if(i==0)
             {
                 if(input.at<uchar>(i,j)>value1&&input.at<uchar>(i,j)<value2)
                 {
                     dp[i]=1;
                 }
                 else
                 {
                     dp[i]=0;
                 }
             }
             else
             {
                 if(input.at<uchar>(i,j)>value1&&input.at<uchar>(i,j)<value2)
                 {
                     dp[i]=dp[i-1]+1;
                 }
                 else
                 {
                     dp[i]=0;
                     if(dp[i-1]>thre1&&dp[i-1]<thre2)
                     {
                         for(int k=0;k<dp[i-1];k++)
                         {
                             num[i-1-k]=1;
                         }

                     }
                 }

             }
         }
         for(int i=0;i<height;i++)
         {
             if(num[i])
             output.at<uchar>(i,j)=255;
             else
             output.at<uchar>(i,j)=0; 


         }

     }
 }
 int main(int argc, char **argv)
 {
     vector<double> p[640];
     vector<double> pp[640];
     vector<double> depth[1000];
     vector<double> worldx[1000];
     vector<double> obstaclex;
     vector<double> obstacley;
     double obstacle[640];
     cv::namedWindow("BarValueThres");
     //cv::namedWindow("video");
     string ss("");
     cv::VideoCapture videoCapture(0);
     videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
     videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
     videoCapture.set(CV_CAP_PROP_EXPOSURE, -6);
     Mat frame;
     Mat distortframe;
     int index = 0;
     //ÏÔÊŸÊÓÆÁ
     char c = 0;
     updatePara();

     ros::init(argc, argv, "talker");
     // %EndTag(INIT)%

     /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
     //create bar
     createTrackbar(trackbar_value1,
                    "BarValueThres", &threshold_value1,
                    255, Threshold_Demo);

     createTrackbar(trackbar_value2,
                    "BarValueThres", &threshold_value2,
                    255, Threshold_Demo);
     createTrackbar(trackbar_value3,
                    "BarValueThres", &threshold_value3,
                    180, Threshold_Demo);

     createTrackbar(trackbar_value4,
                    "BarValueThres", &threshold_value4,
                    180, Threshold_Demo);
     createTrackbar(trackbar_value5,
                    "BarValueThres", &threshold_value5,
                    1000, Threshold_Demo);
     //updatePara();
     //

// %Tag(NODEHANDLE)%
ros::NodeHandle n;
// %EndTag(NODEHANDLE)%
// %Tag(PUBLISHER)%
ros::Publisher chatter_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);//const std::string& topic, uint32_t queue_size
// %EndTag(PUBLISHER)%
//ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
// %Tag(LOOP_RATE)%
ros::Publisher chatter_pub1 = n.advertise<sensor_msgs::LaserScan>("obstacle", 10);
ros::Subscriber odom_sub=n.subscribe< nav_msgs::Odometry>("odom",1000,chatterCallback);
ros::Rate loop_rate(10);//Rate(double frequency)
// %EndTag(LOOP_RATE)%

  //ros::NodeHandle nodeHandler2;
  //ros::Subscriber sub2 = nodeHandler2.subscribe("msg_odometer", 1,chatterCallback);
  /**
  * A count of how many messages we have sent. This is used to create
  * a unique string for each message.
  */
  // %Tag(ROS_OK)%
  sleep(3);
  int count = 0;
  int flag=1;
  int calibflag=0;
  while (ros::ok())
  {
      ros::spinOnce();
      //loop_rate.sleep();
      para[4]=threshold_value5;
      //distorPara[1]=-threshold_value5/1000.0;
      //distorPara[0]=threshold_value5/1000.0;
      //cout<<para[1]<<endl;
      updatePara();
      cv::Mat_<double> posOdo(1,3);
      //posOdo.release();
      {
          boost::mutex::scoped_lock lock(mMutexOdoBuffer);
          if(odoBuffer.size()>0){
              list<nav_msgs::Odometry>::iterator bestOdo;
              double bestErr=1E10;
              double picTime=ros::Time::now().toSec();
              for(list<nav_msgs::Odometry>::iterator itb=odoBuffer.begin();itb!=odoBuffer.end();++itb){
                  double err=abs(itb->header.stamp.toSec()-picTime);
                  if(err<bestErr){
                      bestErr=err;
                      bestOdo=itb;
                  }
              }
              posOdo<<bestOdo->pose.pose.position.x,bestOdo->pose.pose.position.y,bestOdo->pose.pose.position.z;
              if(bestErr<0.01)
              odoBuffer.erase(odoBuffer.begin(),bestOdo);

              if(odoBuffer.size()>20)
              odoBuffer.erase(odoBuffer.begin(),bestOdo);
              //cout<<"ÏÖÔÚµÄŽúÂë£º"<<odoBuffer.size()<<" "<<bestErr<<endl;
          }
      } 

      if(posOdo.empty()) //posOdo.empty()
      {
          //loop_rate.sleep();
          //continue;
      }
      //cout<<"ododata:"<<posOdo<<endl;
      /*else{
      continue;
      loop_rate.sleep();
  }*/

      videoCapture >> frame;


      if (!frame.data)
      continue;
      Mat gray,gray1;
      cvtColor(frame, gray, CV_RGB2GRAY);
  //inRange(gray, threshold_value1, threshold_value2, gray1);
  myRange(gray,gray1,threshold_value3,256,threshold_value1,threshold_value2);
  imshow("myrange",gray1);
  //line detect
  Mat roiImageH;
  //cvtColor(gray1, roiImageH, CV_RGB2HSV);//just input your image here
  cvtColor(gray1, gray1, CV_GRAY2RGB);
  cvtColor(gray1, roiImageH, CV_RGB2HSV);
  vector<Mat> splited;
  split(roiImageH, splited);
  Mat image = splited[2];
  Ptr<LineSegmentDetector> lsd_std = createLineSegmentDetectorPtr(LSD_REFINE_STD);
  //double start = double(getTickCount());
  vector<Vec4i> lines_std,out_line;


        lsd_std->detect(image, lines_std);
        //filter lines
        lsd_std->filterOutAngle(lines_std, out_line,threshold_value3, threshold_value4);
        //


        for (int k = 0; k < 640; k++)
        {
            vector<double> getmin;
            vector<double> getmin2;
            p[k].clear();
            pp[k].clear();
            obstacle[k]=0;
            double x = k, y;
            for (int i = 0; i < out_line.size(); i++)
            {
                double x1 = out_line[i][0];
                double y1 = out_line[i][1];
                double x2 = out_line[i][2];
                double y2 = out_line[i][3];
                if (k >= x1&&k <= x2 || k >= x2&&k <= x1)
                {


                    y = (x - x2) / (x1 - x2)*(y1 - y2) + y2;
                    if(y>=430)
                    {
                        getmin2.push_back(y);
                    }
                    else
                    getmin.push_back(y);
                }


                //circle(frame, Point2f(x1, y1), 3, Scalar(255, 0, 0));
                //circle(frame, Point2f(x2, y2), 3, Scalar(255, 0, 0));
            }
            sort(getmin.begin(), getmin.end(), my_cmp);
            sort(getmin2.begin(), getmin2.end(), my_cmp);

            if (getmin.size() >= 4)
            {
                y = (getmin[0] + getmin[1]) / 2;
                p[k].push_back(y);
                y = (getmin[2] + getmin[3]) / 2;
                p[k].push_back(y);
            }
            else if (getmin.size() > 1)
            {
                y = (getmin[0] + getmin[1]) / 2;
                p[k].push_back(y);
            }
            else if (getmin.size() == 1)
            {
                p[k].push_back(getmin[0]);
            }
            else
            p[k].push_back(0);


            if(getmin2.size()==2)
            {
                obstacle[k] = (getmin2[0] + getmin2[1]) / 2;
            }
            else if(getmin2.size()==1)
            obstacle[k]=getmin2[0];

        }
        int ccount=0;
        double mmax=0;
        int l=640,r=0;
        const int ll=195,rr=410;
        int clong=0,cmax=0;
        int clongtre=10;
        bool obsCon[640];
	obstaclex.clear();
	obstacley.clear();
        for(int k=ll;k<rr;k++)
        {
            if(obstacle[k]!=0||k==rr-1)
            {
                if(k<l)
                l=k;
                if(k>r)
                r=k;
                if(clong>cmax)
                cmax=clong;
                if(clong>clongtre)
                {
                    for(int ii=1;ii<=clong;ii++ )
                    {
                        int num=k-ii;
                        obstaclex.push_back((num-ll)*1.0/(rr-ll)*0.3-0.15);
                        obstacley.push_back(0.2); 
                    }
                }
                clong=0;
                if(obstacle[k-1]!=0)
                {
                    double t=obstacle[k]-obstacle[k-1];
                    t=fabs(t);
                    if(t>mmax)
                    mmax=t;

                }
                ccount++;
            }
            else
            clong++;
        }
        bool ob=false;
        if(cmax>clongtre)//ccount<rr-ll-15||mmax>8||
        {
            sensor_msgs::LaserScan scan_msg1;
            scan_msg1.header.frame_id="obstacle";
            scan_msg1.header.stamp = ros::Time::now();
            chatter_pub1.publish(scan_msg1);
        }
        cout<<"num:"<<ccount<<endl<<"max:"<<mmax<<" clong:"<<cmax<<endl;
        cout<<"l:"<<l<<" "<<"r:"<<r<<endl;
        //wrongPointDetect(p);
        Mat pic(1000, 1000, CV_8UC3);
        int count1 = 0;
        const int nn=0;
        for (int k = nn; k < 640-nn; k++)
        {

            depth[k].clear();
            worldx[k].clear();
            for (int i = 0; i < p[k].size(); i++)
            {
                double a, b;
                circle(frame, Point2f(k, p[k][i]), 3, Scalar(255, 0, 0));
                Mat p_origin, p_after;
                double aa[2] = { k, p[k][i] };
                p_origin.push_back(Mat(1, 1, CV_64FC2, aa));
                undistortPoints(p_origin, p_after, cameraMatrix, distMatrix,cv::Mat(),cameraMatrix);
                //cout << cameraMatrix << endl << distMatrix << endl;
                //cout << p_origin << endl;

                double x0 = p_after.at<double>(0, 0);
                double y0 = p_after.at<double>(0, 1);
                //double x0 = p_origin.at<double>(0, 0);
                //double y0 = p_origin.at<double>(0, 1);

                //double x0 = p_after.at<double>(0, 0);
                //double y0 = p_after.at<double>(0, 1);
                //double fx = 388.2391, cx = 307.5625, fy = 385.5123, cy = 269.5769;
                //x0 = x0*fx + cx;
                //y0 = y0*fy + cy;

                //cout << x0<<" "<<y0 << endl;

                //getPoint3D( k, p[k][i], a, b);
                getPoint3D( x0, y0, a, b);
                //b=b*1.1176-158.9187;
                //a=a*1.1176;
                depth[k].push_back(b);
                worldx[k].push_back(a);
                count1++;
            }




            //cout << a << endl << b << endl;
            /*if (p[k] == 0)
            continue;
            circle(pic, Point2f(a/10+500, b/10), 3, Scalar(255, 0, 0));*/



        }
        /*imshow("map", pic);*/

        //ÀûÓÃÏßÐÔÓÅ»¯ÏµÊýœøÐÐÓÅ»¯
        Mat x = Mat(1, 640, CV_64FC1, depth);
        //Mat result;
        //Mat A, B;
        //result = aa.mul(x).mul(x) + bb.mul(x) + cc;
        //result = aa.mul(x) + bb;
        for (int j = nn; j < 640-nn; j += 50)
        {
            //cout << result.at<double>(0, j) << " ";
            for (int i = 0; i < p[j].size(); i++)
            {
                cout << "[" << worldx[j][i] << "," << depth[j][i] << "] ";
                circle(frame, Point2f(j, p[j][i]), 5, Scalar(0, 255, 0));
            }

        }
        for (int j = nn; j < 640-nn; j++)
        {
            if (p[j][0] == 0)
            continue;
            for (int i = 0; i < worldx[j].size(); i++)
            {
                //circle(pic, Point2f(worldx[j][i] / 5 +600, result.at<double>(0, j) / 5+300), 3, Scalar(255, 0, 0));
                circle(pic, Point2f(worldx[j][i] / 6 + 600, depth[j][i] / 6 + 300), 3, Scalar(255, 0, 0));
            }

        }
        circle(pic, Point2f( 0 + 600, 0 + 300), 10, Scalar(0, 0, 255));
        flip(pic, pic, 0);
        imshow("map", pic);
        cout << endl;
        //cout << endl;

        //show line
        Mat drawnLines(frame);
        //lsd_std->drawSegments(drawnLines, out_line);
        //imshow("Standard refinement", drawnLines);

        imshow("rgb", frame);
        //imshow("video", gray);
        //imshow("extract", gray1);
        //imshow("disvideo", distortframe);
        if (c == 32){ //c==32
                     /*	imwrite((num2str(index) + ".jpg").c_str(), frame);
                     cout << index << endl;
                     ++index;
                     Sleep(2000);*/
                     static ofstream outfile;
                     if (!outfile.is_open()) {
                         cout << "not open" << endl;
                         outfile.open("depth.txt", ios::out);//ÎÄŒþÃûžÄ³É×ÔŒºµÄ
                     }
                     for (int i = 0; i < 639; i++)
                     {
                         outfile << depth[i][0] << '\t';
                     }
                     outfile << depth[639][0] << endl;
                    }

        c = cvWaitKey(30);
        // %EndTag(ROS_OK)%
        /**
        * This is a message object. You stuff it with data, and then publish it.
        */
        // %Tag(FILL_MESSAGE)%
        const int lasernum=500;
        std_msgs::String msg;
        sensor_msgs::LaserScan scan_msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
        scan_msg.header.frame_id="laser";
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.angle_min = -3.1415926/4;
        scan_msg.angle_max = 3.1415926/4;
        scan_msg.angle_increment = 3.1415926/2/lasernum;
        scan_msg.time_increment = 0.02;
        scan_msg.scan_time = 0.02;
        scan_msg.range_min = 0.1;
        scan_msg.range_max = 5;
        scan_msg.ranges.assign(lasernum, std::numeric_limits<float>::quiet_NaN());
        for(int i=0;i<lasernum;i++)
        {
            scan_msg.ranges[i]=0;
        }
        for(int i=0;i<640;i++)
        {

            for(int j=0;j<depth[i].size();j++)
            {
                double c=sqrt(depth[i][j]*depth[i][j]+worldx[i][j]*worldx[i][j]);
                double angle=3.1415926/2-acos(worldx[i][j]/c);
                //cout<<"angle:"<<angle<<endl;
                angle=angle+3.1415926/4;
                //if(i%20==0)
                //cout<<"angle:"<<angle*180/3.1415926<<endl;
                //c=c*15/13;
                int k=angle/scan_msg.angle_increment;
                if(p[i][0]!=0&&k>=0&&k<lasernum)
                scan_msg.ranges[lasernum-k-1]=c/1000;  
            }

        }
        for(int i=0;i<obstaclex.size();i++)
        {
            double c=sqrt(obstaclex[i]*obstaclex[i]+obstacley[i]*obstacley[i]);
            double angle=3.1415926/2-acos(obstaclex[i]/c);
            angle=angle+3.1415926/4;
            int k=angle/scan_msg.angle_increment;
            if(k>=0&&k<lasernum)
            scan_msg.ranges[lasernum-k-1]=c;  
        }
        // %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor above.
    */
    // %Tag(PUBLISH)%
    /* if(flag)
    {

      flag=0;
      continue;
  }*/
  static ofstream outfile;
  if (!outfile.is_open()) {
      cout << "not open" << endl;
      outfile.open("laserodom5.2d", ios::out);//ÎÄŒþÃûžÄ³É×ÔŒºµÄ
      outfile<<"LaserOdometryLog"<<endl<<"#Created by Master"<<endl;
      outfile<<"version: 1"<<endl;
      outfile<<"sick1pose: 0 0 0"<<endl;
      outfile<<"sick1conf: -45 45 "<<lasernum<<endl;

  }
  time_t t=time(0);
  char tmp[64];
  strftime(tmp,sizeof(tmp),"%H:%M:%S",localtime(&t));
  outfile<<"scan1Id: "<<count+1<<endl;
  outfile<<"time: "<<tmp<<endl;
  outfile<<"robot: "<<(int)(posOdo.at<double>(0,0)*1000)<<" "<<(int)(posOdo.at<double>(0,1)*1000)<<" "<<(int)(posOdo.at<double>(0,2)*180/3.1415926)<<endl;
  outfile<<"sick1: ";
  for (int i = 0; i < lasernum; i++)
  {
      outfile<<(int)(scan_msg.ranges[i]*1000)<<" ";
  }
  outfile <<endl;

    chatter_pub.publish(scan_msg);
    // %EndTag(PUBLISH)%


    //

    //tf transform

    static tf::TransformBroadcaster laser_broadcaster;
    tf::Transform laser_transform;
    laser_transform.setOrigin( tf::Vector3(0, 0, 0.05) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    laser_transform.setRotation(q);
    laser_broadcaster.sendTransform(tf::StampedTransform(laser_transform, ros::Time::now(), "base_link", "laser"));
    static tf::TransformBroadcaster map_broadcaster;
    tf::Transform map_transform;   
    map_transform.setOrigin( tf::Vector3(0, 0, 0) );
    //tf::Quaternion q;
    q.setRPY(0, 0, 0);
    map_transform.setRotation(q);
    //map_broadcaster.sendTransform(tf::StampedTransform(map_transform, ros::Time::now(), "map", "odom"));




      //ROS_INFO("%s", "111");
      //fake odom publish
      /*
      nav_msgs::Odometry odometerData;

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
                odom_pub.publish(odometerData);*/
                // %Tag(SPINONCE)%
                //ros::spinOnce();
                // %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
++count;
}
return 0;
}
// %EndTag(FULLTEXT)%
