#pragma once

#include <stdexcept>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "state_machine.hpp"

#include "cpprest/asyncrt_utils.h"
#include "cpprest/http_listener.h"
#include "cpprest/json.h"
#include "cpprest/uri.h"

class CoreNode
{
    ros::NodeHandle m_node_handle;

    ros::ServiceClient m_service_client;
    
	Context m_context;

	FSM::Instance m_state_machine;

    web::http::experimental::listener::http_listener m_listener;

public:
    CoreNode(ros::NodeHandle const &node_handle);
    ~CoreNode();

    void getHandler(web::http::http_request message);
    void postHandler(web::http::http_request message);

    void update();
};
