#include "core_node.hpp"

CoreNode::CoreNode(ros::NodeHandle const &node_handle) : m_node_handle(node_handle), m_state_machine(m_context), m_listener("http://0.0.0.0:8080")
{
    m_listener.support(web::http::methods::GET, std::bind(&CoreNode::getHandler, this, std::placeholders::_1));
    m_listener.support(web::http::methods::POST, std::bind(&CoreNode::postHandler, this, std::placeholders::_1));

    m_listener.open().wait();
}

CoreNode::~CoreNode()
{
    m_listener.close().wait();
}

void CoreNode::getHandler(web::http::http_request request)
{
    auto paths = web::uri::split_path(web::uri::decode(request.relative_uri().path()));

    if (paths[0] == "get_state") {
        auto response = web::json::value::object();

        //response["state"] = ;

        request.reply(web::http::status_codes::OK, response);
    }
    else {
        request.reply(web::http::status_codes::BadRequest);
    }
}

void CoreNode::postHandler(web::http::http_request request)
{
    auto paths = web::uri::split_path(web::uri::decode(request.relative_uri().path()));

    auto query = web::uri::split_query(web::uri::decode(request.request_uri().query()));

    if (paths[0] == "set_state") {
        if (query["state"] == "idle") {
            m_state_machine.changeTo<Idle>();

            request.reply(web::http::status_codes::OK);
        }
        else if (query["state"] == "manual") {
            m_state_machine.changeTo<Manual>();

            request.reply(web::http::status_codes::OK);
        }
        else if (query["state"] == "automatic") {
            m_state_machine.changeTo<Automatic>();

            request.reply(web::http::status_codes::OK);
        }
    }
    else {
        request.reply(web::http::status_codes::BadRequest);
    }
}

void CoreNode::update()
{
    m_state_machine.update();
}

int main(int argc, char **argv)
{
    try {
        ros::init(argc, argv, "core");

        ros::NodeHandle node_handle("~");

        CoreNode core_node(node_handle);
        
        while (ros::ok()) {
            core_node.update();

            ros::spinOnce();
        }

        return 0;
    }
    catch (std::exception const &exception) {
        ROS_ERROR("%s", exception.what());

        return 1;
    }
}
