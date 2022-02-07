#include "stepper_motor_node.hpp"

StepperMotorNode::StepperMotorNode(ros::NodeHandle const &node_handle) : m_node_handle(node_handle)
{
    std::string path_name;
    std::string set_stepper_motor_service;

    if (!m_node_handle.getParam("path_name", path_name)) {
        throw std::runtime_error("path_name not provided");
    }

    if (!m_node_handle.getParam("set_stepper_motor_service", set_stepper_motor_service)) {
        throw std::runtime_error("set_stepper_motor_service not provided");
    }

    m_set_stepper_motor_server = m_node_handle.advertiseService(set_stepper_motor_service, &StepperMotorNode::onSetStepperMotor, this);

    m_send_command_client = m_node_handle.serviceClient<serial_command_client::send_command>(path_name + "/send_command");
}

bool StepperMotorNode::onSetStepperMotor(stepper_motor::set_stepper_motor::Request &request, stepper_motor::set_stepper_motor::Response &response)
{
    serial_command_client::send_command send_command_srv;

    send_command_srv.request.command = 1;

    send_command_srv.request.buffer.resize(sizeof(uint32_t) + sizeof(uint32_t) + sizeof(int32_t));

    std::memcpy(&send_command_srv.request.buffer[0], &request.stepper_motor_id, sizeof(uint32_t));
    std::memcpy(&send_command_srv.request.buffer[sizeof(uint32_t)], &request.speed, sizeof(uint32_t));
    std::memcpy(&send_command_srv.request.buffer[sizeof(uint32_t) + sizeof(uint32_t)], &request.step, sizeof(int32_t));

    if (m_send_command_client.call(send_command_srv)) {
        if (send_command_srv.response.status == 0) {
            return true;
        }
    }

    return false;
}

int main(int argc, char **argv)
{
    try {
        ros::init(argc, argv, "stepper_motor");

        ros::NodeHandle node_handle("~");

        StepperMotorNode stepper_motor_node(node_handle);
        
        while (ros::ok()) {
            ros::spinOnce();
        }

        return 0;
    }
    catch (std::exception const &exception) {
        ROS_ERROR("%s", exception.what());

        return 1;
    }
}
