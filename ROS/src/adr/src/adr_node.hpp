#pragma once

#include <stdexcept>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "adr/set_state.h"
#include "adr/get_state.h"

#include "finite_state_machine.hpp"

class ADRNode
{
    ros::NodeHandle m_node_handle;
    
	Context m_context;

	FiniteStateMachine::Instance m_finite_state_machine;

    ros::ServiceServer m_get_state_server;
    ros::ServiceServer m_set_state_server;

public:
    ADRNode(ros::NodeHandle const &node_handle);
    ~ADRNode();

    void update();

private:
    bool onGetState(adr::get_state::Request &request, adr::get_state::Response &response);
    bool onSetState(adr::set_state::Request &request, adr::set_state::Response &response);
};
