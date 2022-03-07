#pragma once

#include <stdexcept>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "asdr/set_state.h"
#include "asdr/get_state.h"
#include "asdr/set_velocity.h"

#include "finite_state_machine.hpp"

class ASDRNode
{
    ros::NodeHandle m_node_handle;
    
	Context m_context;

	FiniteStateMachine::Instance m_finite_state_machine;

    ros::ServiceServer m_get_state_server;
    ros::ServiceServer m_set_state_server;
    ros::ServiceServer m_set_velocity_server;

public:
    ASDRNode(ros::NodeHandle const &node_handle);

    void update();

private:
    bool onGetState(asdr::get_state::Request &request, asdr::get_state::Response &response);
    bool onSetState(asdr::set_state::Request &request, asdr::set_state::Response &response);
    bool onSetVelocity(asdr::set_velocity::Request &request, asdr::set_velocity::Response &response);
};
