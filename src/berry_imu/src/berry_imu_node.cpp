#include "berry_imu_node.hpp"

BerryIMUNode::BerryIMUNode(int32_t const &FS_G, int32_t const &FS_XL, int32_t const &FS_M) : ros::NodeHandle(""), m_berry_imu(FS_G, FS_XL, FS_M)
{
    std::string imu_topic;
    std::string mag_topic;

    if (!getParam("imu_topic", imu_topic)) {
        imu_topic = "imu/data_raw";
    }

    if (!getParam("mag_topic", mag_topic)) {
        mag_topic = "imu/mag";
    }

    m_imu_publisher = advertise<sensor_msgs::Imu>(imu_topic, 100);
    m_magnetic_field_publisher = advertise<sensor_msgs::MagneticField>(mag_topic, 100);
}

void BerryIMUNode::publish()
{	
    double gyr[3];
    double acc[3];
    double mag[3];

    m_berry_imu.readGyr(gyr);
    m_berry_imu.readAcc(acc);
    m_berry_imu.readMag(mag);

    sensor_msgs::Imu imu_msg;

    imu_msg.angular_velocity.x = gyr[0];
    imu_msg.angular_velocity.y = gyr[1];
    imu_msg.angular_velocity.z = gyr[2];

    imu_msg.angular_velocity_covariance[0] = 1.0;
    imu_msg.angular_velocity_covariance[1] = 0.0;
    imu_msg.angular_velocity_covariance[2] = 0.0;
    imu_msg.angular_velocity_covariance[3] = 0.0;
    imu_msg.angular_velocity_covariance[4] = 1.0;
    imu_msg.angular_velocity_covariance[5] = 0.0;
    imu_msg.angular_velocity_covariance[6] = 0.0;
    imu_msg.angular_velocity_covariance[7] = 0.0;
    imu_msg.angular_velocity_covariance[8] = 1.0;

    imu_msg.linear_acceleration.x = acc[0];
    imu_msg.linear_acceleration.y = acc[1];
    imu_msg.linear_acceleration.z = acc[2];

    imu_msg.linear_acceleration_covariance[0] = 1.0;
    imu_msg.linear_acceleration_covariance[1] = 0.0;
    imu_msg.linear_acceleration_covariance[2] = 0.0;
    imu_msg.linear_acceleration_covariance[3] = 0.0;
    imu_msg.linear_acceleration_covariance[4] = 1.0;
    imu_msg.linear_acceleration_covariance[5] = 0.0;
    imu_msg.linear_acceleration_covariance[6] = 0.0;
    imu_msg.linear_acceleration_covariance[7] = 0.0;
    imu_msg.linear_acceleration_covariance[8] = 1.0;

    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "berry_imu";

    m_imu_publisher.publish(imu_msg);
    
    sensor_msgs::MagneticField magnetic_field_msg;

    magnetic_field_msg.magnetic_field.x = mag[0];
    magnetic_field_msg.magnetic_field.y = mag[1];
    magnetic_field_msg.magnetic_field.z = mag[2];

    magnetic_field_msg.magnetic_field_covariance[0] = 1.0;
    magnetic_field_msg.magnetic_field_covariance[1] = 0.0;
    magnetic_field_msg.magnetic_field_covariance[2] = 0.0;
    magnetic_field_msg.magnetic_field_covariance[3] = 0.0;
    magnetic_field_msg.magnetic_field_covariance[4] = 1.0;
    magnetic_field_msg.magnetic_field_covariance[5] = 0.0;
    magnetic_field_msg.magnetic_field_covariance[6] = 0.0;
    magnetic_field_msg.magnetic_field_covariance[7] = 0.0;
    magnetic_field_msg.magnetic_field_covariance[8] = 1.0;

    magnetic_field_msg.header.stamp = ros::Time::now();
    magnetic_field_msg.header.frame_id = "berry_imu";

    m_magnetic_field_publisher.publish(magnetic_field_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "berry_imu_node");

    BerryIMUNode berry_imu_node;
    
    while (ros::ok()) {
        berry_imu_node.publish();

        ros::spinOnce();
    }

    return 0;
}
