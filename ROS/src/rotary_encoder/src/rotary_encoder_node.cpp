#include "rotary_encoder_node.hpp"

RotaryEncoderNode::RotaryEncoderNode(ros::NodeHandle const &node_handle) : m_node_handle(node_handle)
{
    m_get_rotary_encoder_server = m_node_handle.advertiseService(ros::names::resolve("get_rotary_encoder"), &RotaryEncoderNode::onGetRotaryEncoder, this);

    m_send_command_client = m_node_handle.serviceClient<serial_command_client::send_command>(ros::names::resolve("send_command"));
}

bool RotaryEncoderNode::onGetRotaryEncoder(rotary_encoder::get_rotary_encoder::Request &request, rotary_encoder::get_rotary_encoder::Response &response)
{
    serial_command_client::send_command send_command_srv;

    send_command_srv.request.command = 0;

    send_command_srv.request.buffer.resize(sizeof(uint32_t));

    std::memcpy(&send_command_srv.request.buffer[0], &request.rotary_encoder_id, sizeof(uint32_t));

    if (m_send_command_client.call(send_command_srv)) {
        if (send_command_srv.response.status == 1 && std::size(send_command_srv.response.buffer) == sizeof(int32_t) + sizeof(int32_t)) {
            std::memcpy(&response.position, &send_command_srv.response.buffer[0], sizeof(int32_t));
            std::memcpy(&response.direction, &send_command_srv.response.buffer[sizeof(int32_t)], sizeof(int32_t));

            return true;
        }
    }

    return false;
}

int main(int argc, char **argv)
{
    try {
        ros::init(argc, argv, "rotary_encoder");

        ros::NodeHandle node_handle("~");

        RotaryEncoderNode rotary_encoder_node(node_handle);
        
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
