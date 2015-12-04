#include "imu_filter.h"


int main(int argc, char **argv)
{

       ros::init(argc, argv, "imu_filter");
       Imu_filter iff;

       ros::spin();
 
      return (0);
}
