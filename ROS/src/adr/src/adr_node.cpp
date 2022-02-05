#include "adr_node.hpp"

ADRNode::ADRNode(ros::NodeHandle const &node_handle) : m_node_handle(node_handle), m_context(node_handle), m_state_machine(m_context)
{
    std::string get_state_service;
    std::string set_state_service;

    if (!m_node_handle.getParam("get_state_service", get_state_service)) {
        throw std::runtime_error("get_state_service not provided");
    }

    if (!m_node_handle.getParam("set_state_service", set_state_service)) {
        throw std::runtime_error("set_state_service not provided");
    }

    m_get_state_server = m_node_handle.advertiseService(get_state_service, &ADRNode::onGetState, this);
    m_set_state_server = m_node_handle.advertiseService(set_state_service, &ADRNode::onSetState, this);
}

ADRNode::~ADRNode()
{
}

void ADRNode::update()
{
    m_state_machine.update();
}

bool ADRNode::onGetState(adr::get_state::Request &request, adr::get_state::Response &response)
{
    if (m_state_machine.isActive<Idle>()) {
        response.state = "Idle";

        return true;
    }
    else if (m_state_machine.isActive<Manual>()) {
        response.state = "Manual";

        return true;
    }
    else if (m_state_machine.isActive<Automatic>()) {
        response.state = "Automatic"; 

        return true;
    }
    else if (m_state_machine.isActive<Discover>()) {
        response.state = "Discover";

        return true;
    }
    else if (m_state_machine.isActive<Observe>()) {
        response.state = "Observe";

        return true;
    }
    else if (m_state_machine.isActive<Explore>()) {
        response.state = "Explore";

        return true;
    }
    else if (m_state_machine.isActive<Disinfect>()) {
        response.state = "Disinfect";

        return true;
    }
    else if (m_state_machine.isActive<Plan>()) {
        response.state = "Plan";

        return true;
    }
    else if (m_state_machine.isActive<LightOn>()) {
        response.state = "LightOn";

        return true;
    }
    else if (m_state_machine.isActive<Navigate>()) {
        response.state = "Navigate";

        return true;
    }
    else if (m_state_machine.isActive<LightOff>()) {
        response.state = "LightOff";

        return true;
    }

    return false;
}

bool ADRNode::onSetState(adr::set_state::Request &request, adr::set_state::Response &response)
{
    if (request.state == "Idle") {
        m_state_machine.changeTo<Idle>();

        return true;
    }
    else if (request.state == "Manual") {
        m_state_machine.changeTo<Manual>();

        return true;
    }
    else if (request.state == "Automatic") {
        m_state_machine.changeTo<Automatic>();

        return true;
    }
    
    return false;
}

int main(int argc, char **argv)
{
    try {
        ros::init(argc, argv, "adr");

        ros::NodeHandle node_handle("~");

        ADRNode adr_node(node_handle);
        
        while (ros::ok()) {
            adr_node.update();

            ros::spinOnce();
        }

        return 0;
    }
    catch (std::exception const &exception) {
        ROS_ERROR("%s", exception.what());

        return 1;
    }
}
