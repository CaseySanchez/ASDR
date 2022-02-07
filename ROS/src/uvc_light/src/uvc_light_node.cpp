#include "uvc_light_node.hpp"

UVCLightNode::UVCLightNode(ros::NodeHandle const &node_handle) : m_node_handle(node_handle)
{
    std::string set_uvc_light_service;

    if (!m_node_handle.getParam("set_uvc_light_service", set_uvc_light_service)) {
        throw std::runtime_error("set_uvc_light_service not provided");
    }

    m_set_uvc_light_server = m_node_handle.advertiseService(ros::names::resolve(set_uvc_light_service), &UVCLightNode::onSetUVCLight, this);

    m_send_command_client = m_node_handle.serviceClient<serial_command_client::send_command>(ros::names::resolve("send_command"));
}

bool UVCLightNode::onSetUVCLight(uvc_light::set_uvc_light::Request &request, uvc_light::set_uvc_light::Response &response)
{
    serial_command_client::send_command send_command_srv;

    send_command_srv.request.command = 1;

    send_command_srv.request.buffer.resize(sizeof(bool));

    std::memcpy(&send_command_srv.request.buffer[0], &request.state, sizeof(bool));

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
        ros::init(argc, argv, "uvc_light");

        ros::NodeHandle node_handle("~");

        UVCLightNode uvc_light_node(node_handle);
        
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
