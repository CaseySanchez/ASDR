#pragma once

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include "berry_imu.hpp"

class BerryIMUNode : public ros::NodeHandle
{
    BerryIMU m_berry_imu;
    
    ros::Publisher m_imu_publisher;
    ros::Publisher m_magnetic_field_publisher;

public:
    BerryIMUNode(int32_t const &FS_G = FS_G_2000, int32_t const &FS_XL = FS_XL_16, int32_t const &FS_M = FS_M_16);
    
    void publish();
};
