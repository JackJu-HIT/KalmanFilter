/*
*Author:Jack Ju
*HIT
*卡尔曼滤波算法实现
*利用前面时刻的加速度与当前时刻加速度相同作为预测
*传感器当前时刻的测量结果为测量。
*20200802
*/
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "Eigen/Dense"
//#include "sensor_msgs/LaserScan.h"

double a_x;
double a_y;
double a_z;


 Eigen::MatrixXd X=Eigen::MatrixXd(3,1);

using namespace std;
class kalman_filter
{
private:
    
    /* data */
 //Eigen::MatrixXd x;//要处理的状态变量
 Eigen::MatrixXd A;//系统状态矩阵
 Eigen::MatrixXd P;//协方差
 Eigen::MatrixXd Q;//测量过程噪音（预测）
 Eigen::MatrixXd R;//真实传感器噪音
 Eigen::MatrixXd H;//测量矩阵
 //Eigen::MatrixXd x = Eigen::MatrixXd(3,1);
 // Eigen::MatrixXd z = Eigen::MatrixXd(3,1);

 bool isinitized = false;
public:
    kalman_filter(/* args */);
    Eigen::MatrixXd predictAndupdate( Eigen::MatrixXd x,Eigen::MatrixXd z);
    //void update();
    ~kalman_filter();
};
Eigen::MatrixXd kalman_filter::predictAndupdate(Eigen::MatrixXd x,Eigen::MatrixXd z)
{
  if(!isinitized)
  {
  P=Eigen::MatrixXd(3,3);
  P<<1,0,0,
     0,1,0,
     0,0,1;
  isinitized=true;
  }
  x=A*x;
  P=A*P*(A.transpose())+Q;
  Eigen::MatrixXd  K=P*(H.transpose())*((H*P*(H.transpose())+R).inverse());
  x=x+K*(z-H*x);
  int x_size = x.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P=(I-K*H)*P;
  return x;

}

kalman_filter::kalman_filter(/* args */)
{
//参数初始化设置
   A=Eigen::MatrixXd(3,3);
   A<<1,0,0,
      0,1,0,
      0,0,1;
   H=Eigen::MatrixXd(3,3);
   H<<1,0,0,
      0,1,0,
      0,0,1;
    Q=Eigen::MatrixXd(3,3);
    Q<<0.1,0,0,
      0,0.1,0,
      0,0,0.1;
    R=Eigen::MatrixXd(3,3);
    R<<9,0,0,
       0,0.09,0,
       0,0,0.09;
}

kalman_filter::~kalman_filter()
{
}



class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
   
 
    //Topic you want to subscribe
   imu_info_sub = n.subscribe("/imu", 10, &SubscribeAndPublish::imuInfoCallback,this);
   IMU_kalman_pub = n.advertise<sensor_msgs::Imu>("/imu_kalman", 10);
 
  }
 

 void imuInfoCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 第一次将接收到的消息打印出来
    if(first)
    {
   double a_x_m=msg->linear_acceleration.x;
   double a_y_m=msg->linear_acceleration.y;
   double a_z_m=msg->linear_acceleration.z;
   X<<a_x_m,a_y_m,a_z_m;
   //第一次不用卡尔曼滤波，直接发布出去，
   sensor_msgs::Imu M;
    M.linear_acceleration.x=a_x_m;
    M.linear_acceleration.y=a_y_m;
    M.linear_acceleration.z=a_z_m;
    M.header=msg->header;
    IMU_kalman_pub.publish(M);
    first=false;

    }
    else
    {
      /* code */
  double a_x_m=msg->linear_acceleration.x;
  double a_y_m=msg->linear_acceleration.y;
  double a_z_m=msg->linear_acceleration.z;
  Eigen::MatrixXd z;
  z=Eigen::MatrixXd(3,1);
  z<<a_x_m,a_y_m,a_z_m;
  kalman_filter kf;
  Eigen::MatrixXd x_new=kf.predictAndupdate(X,z);
  X=z;
    //ax_pre=a_x_m;
   // ay_pre=a_y_m;
   
  a_x=x_new(0,0);
  a_y=x_new(1,0);
  a_z=x_new(2,0);
  sensor_msgs::Imu M;
  M.linear_acceleration.x=a_x;
  M.linear_acceleration.y=a_y;
  M.linear_acceleration.z=a_z;
  M.header=msg->header;
  IMU_kalman_pub.publish(M);
 
    }
    
  
}

 
private:
  ros::NodeHandle n; 
  ros::Publisher IMU_kalman_pub;
  ros::Subscriber imu_info_sub;
  bool first=true;
  //Eigen::MatrixXd x;
 
 
};//End of class SubscribeAndPublish
 
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");
 
 
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;
 
  ros::spin();
 
  return 0;
}

/**

int main(int argc, char **argv)
{
ros::init(argc, argv, "filter_dealing");

// 创建节点句柄
ros::NodeHandle n;
ros::Publisher IMU_kalman_pub = n.advertise<sensor_msgs::Imu>("/imu_kalman", 10);
  
ros::Subscriber imu_info_sub = n.subscribe("/imu", 10, imuInfoCallback);

 // 循环等待回调函数
    ros::spin();

    return 0;


}
*/