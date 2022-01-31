#include "berry_imu_node.hpp"

BerryIMUNode::BerryIMUNode(ros::NodeHandle const &node_handle) : m_node_handle(node_handle)
{
    std::string path_name;
    std::string imu_topic;
    std::string mag_topic;

    if (!m_node_handle.getParam("path_name", path_name)) {
        throw std::runtime_error("path_name not provided");
    }

    if (!m_node_handle.getParam("imu_topic", imu_topic)) {
        throw std::runtime_error("imu_topic not provided");
    }

    if (!m_node_handle.getParam("mag_topic", mag_topic)) {
        throw std::runtime_error("mag_topic not provided");
    }

    m_imu_publisher = m_node_handle.advertise<sensor_msgs::Imu>(imu_topic, 100);
    m_magnetic_field_publisher = m_node_handle.advertise<sensor_msgs::MagneticField>(mag_topic, 100);

    m_send_command_client = m_node_handle.serviceClient<serial_command_client::send_command>(path_name + "/send_command");
}

void BerryIMUNode::publish()
{
    serial_command_client::send_command send_command_srv;

    send_command_srv.request.command = 0;

    if (m_send_command_client.call(send_command_srv)) {
        if (send_command_srv.response.status == 0 && send_command_srv.response.buffer.size() == sizeof(float) * 3 * 3) {
            float gyr[3];
            float acc[3];
            float mag[3];

            std::memcpy(&gyr[0], &send_command_srv.response.buffer[sizeof(float) * 3 * 0], sizeof(float) * 3);
            std::memcpy(&acc[0], &send_command_srv.response.buffer[sizeof(float) * 3 * 1], sizeof(float) * 3);
            std::memcpy(&mag[0], &send_command_srv.response.buffer[sizeof(float) * 3 * 2], sizeof(float) * 3);

            sensor_msgs::Imu imu_msg;

            imu_msg.orientation.x = 0.0;
            imu_msg.orientation.y = 0.0;
            imu_msg.orientation.z = 0.0;
            imu_msg.orientation.w = 0.0;

            imu_msg.orientation_covariance[0] = -1.0;
            imu_msg.orientation_covariance[1] = 0.0;
            imu_msg.orientation_covariance[2] = 0.0;
            imu_msg.orientation_covariance[3] = 0.0;
            imu_msg.orientation_covariance[4] = 0.0;
            imu_msg.orientation_covariance[5] = 0.0;
            imu_msg.orientation_covariance[6] = 0.0;
            imu_msg.orientation_covariance[7] = 0.0;
            imu_msg.orientation_covariance[8] = 0.0;

            imu_msg.angular_velocity.x = gyr[0];
            imu_msg.angular_velocity.y = gyr[1];
            imu_msg.angular_velocity.z = gyr[2];

            imu_msg.angular_velocity_covariance[0] = 0.01;
            imu_msg.angular_velocity_covariance[1] = 0.0;
            imu_msg.angular_velocity_covariance[2] = 0.0;
            imu_msg.angular_velocity_covariance[3] = 0.0;
            imu_msg.angular_velocity_covariance[4] = 0.01;
            imu_msg.angular_velocity_covariance[5] = 0.0;
            imu_msg.angular_velocity_covariance[6] = 0.0;
            imu_msg.angular_velocity_covariance[7] = 0.0;
            imu_msg.angular_velocity_covariance[8] = 0.01;

            imu_msg.linear_acceleration.x = acc[0];
            imu_msg.linear_acceleration.y = acc[1];
            imu_msg.linear_acceleration.z = acc[2];

            imu_msg.linear_acceleration_covariance[0] = 0.01;
            imu_msg.linear_acceleration_covariance[1] = 0.0;
            imu_msg.linear_acceleration_covariance[2] = 0.0;
            imu_msg.linear_acceleration_covariance[3] = 0.0;
            imu_msg.linear_acceleration_covariance[4] = 0.01;
            imu_msg.linear_acceleration_covariance[5] = 0.0;
            imu_msg.linear_acceleration_covariance[6] = 0.0;
            imu_msg.linear_acceleration_covariance[7] = 0.0;
            imu_msg.linear_acceleration_covariance[8] = 0.01;

            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "berry_imu";

            m_imu_publisher.publish(imu_msg);
            
            sensor_msgs::MagneticField magnetic_field_msg;

            magnetic_field_msg.magnetic_field.x = mag[0];
            magnetic_field_msg.magnetic_field.y = mag[1];
            magnetic_field_msg.magnetic_field.z = mag[2];

            magnetic_field_msg.magnetic_field_covariance[0] = 0.01;
            magnetic_field_msg.magnetic_field_covariance[1] = 0.0;
            magnetic_field_msg.magnetic_field_covariance[2] = 0.0;
            magnetic_field_msg.magnetic_field_covariance[3] = 0.0;
            magnetic_field_msg.magnetic_field_covariance[4] = 0.01;
            magnetic_field_msg.magnetic_field_covariance[5] = 0.0;
            magnetic_field_msg.magnetic_field_covariance[6] = 0.0;
            magnetic_field_msg.magnetic_field_covariance[7] = 0.0;
            magnetic_field_msg.magnetic_field_covariance[8] = 0.01;

            magnetic_field_msg.header.stamp = ros::Time::now();
            magnetic_field_msg.header.frame_id = "berry_imu";

            m_magnetic_field_publisher.publish(magnetic_field_msg);
        }
    }
}

int main(int argc, char **argv)
{
    try {
        ros::init(argc, argv, "berry_imu");

        ros::NodeHandle node_handle("~");

        BerryIMUNode berry_imu_node(node_handle);
        
        while (ros::ok()) {
            berry_imu_node.publish();

            ros::spinOnce();
        }

        return 0;
    }
    catch (std::exception const &exception) {
        ROS_ERROR("%s", exception.what());

        return 1;
    }
}
