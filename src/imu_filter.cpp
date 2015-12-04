#include "imu_filter.h"


Imu_filter::Imu_filter()
{
    //ros::NodeHandle nh;
    ros::NodeHandle pn("~");
    pn.param("topic", p_imu, std::string("fcu/imu"));
     ImuSub = node_.subscribe(p_imu, 50, &Imu_filter::imuMsgCallback,this);
     bias_xpublisher_ = node_.advertise<geometry_msgs::Vector3>("/x_bias",50);
   //  bias_ypublisher_ = nh.advertise<float>("/y_bias", 1, false);
     acc_xpublisher_ = node_.advertise<geometry_msgs::Vector3>("/x_acc",50);
     time_publisher_ = node_.advertise<std_msgs::Float64>("/time", 50);
     calibration_count = 1;
     sampling_count = 1;
}

Imu_filter::~Imu_filter()
{

}

void Imu_filter::imuMsgCallback(const sensor_msgs::Imu& imu_msg_raw)
{
       
      if(imu_msg_raw.linear_acceleration.x<1 && imu_msg_raw.linear_acceleration.x>-1 && imu_msg_raw.linear_acceleration.y<1 && imu_msg_raw.linear_acceleration.y>-1)
      {
          Imu_filter::calibrate_x(imu_msg_raw.linear_acceleration);
      }
      else
      {
          Imu_filter::sampling(imu_msg_raw.linear_acceleration);
      }


}

void Imu_filter::calibrate_x(const geometry_msgs::Vector3& imu_msg_raw)
{
    if((calibration_count%51)==0)
    {
        xbias = a_x/calibration_count;
        ybias = a_y/calibration_count;
        bias.x = xbias;
        bias.y = ybias;
        bias_xpublisher_.publish(bias);
       // bias_ypublisher_.publish(ybias);
       Accu.x=0;
       Accu.y=0;
       acc_xpublisher_.publish(Accu);
       last.data = ros::Time::now().toSec();
	
	 dt.data = last.data - begin.data ;
	 time_publisher_.publish(dt);
        calibration_count = 1;
        a_x = 0;
        a_y = 0;
	 

    }
     if(calibration_count == 1)
     { 
        begin.data = ros::Time::now().toSec();
     }
    

        a_x += imu_msg_raw.x;
        a_y += imu_msg_raw.y;
        calibration_count++;
    



   //return x_Bias;



}

void Imu_filter::sampling(const geometry_msgs::Vector3& acc)
{
    
    
     
     if((sampling_count%51)==0)
    {
          xAccu = acc_x/sampling_count;
          yAccu = acc_y/sampling_count;
          xAccu -= xbias;
          yAccu -= ybias;
          Accu.x = xAccu;
          Accu.y = yAccu;
          acc_xpublisher_.publish(Accu);
         // acc_ypublisher_.publish(yAccu);
          sampling_count = 1;
          acc_x = 0;
          acc_y = 0;
         last.data = ros::Time::now().toSec();
	
	 dt.data = last.data - begin.data ;
	 time_publisher_.publish(dt);
    }
     if(sampling_count == 1)
     { 
        begin.data = ros::Time::now().toSec();
     }
     acc_x += acc.x;
     acc_y += acc.y;
     sampling_count++;
     
    

}


