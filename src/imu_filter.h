#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "ros/ros.h"  
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Float64.h"

class Imu_filter
 {

  public:
     Imu_filter();
     ~Imu_filter();
  
   void imuMsgCallback(const sensor_msgs::Imu& imu_msg_raw);
   void calibrate_x(const geometry_msgs::Vector3& imu_msg_raw);
   void sampling(const geometry_msgs::Vector3& acc);



  protected:
   sensor_msgs::Imu fused_imu_msg_;
   std::string p_imu;
int sampling_count;
int calibration_count;
float xbias;
float ybias;
float xAccu;
float yAccu;
float a_x;
float a_y;
float acc_x;
float acc_y;
std_msgs::Float64 begin;
std_msgs::Float64 last;
ros::Publisher bias_xpublisher_;
ros::Publisher bias_ypublisher_;
ros::Publisher acc_xpublisher_;
ros::Publisher time_publisher_;
ros::NodeHandle node_;
 ros::Subscriber ImuSub;
  std_msgs::Float64 dt; 
geometry_msgs::Vector3 bias;
geometry_msgs::Vector3 Accu;

 };


#endif // IMU_FILTER_H
