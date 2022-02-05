#include "rest_node.hpp"

RESTNode::RESTNode(ros::NodeHandle const &node_handle) : m_node_handle(node_handle), m_listener("http://0.0.0.0:8080")
{
    m_listener.support(web::http::methods::GET, std::bind(&RESTNode::onGet, this, std::placeholders::_1));
    m_listener.support(web::http::methods::POST, std::bind(&RESTNode::onPost, this, std::placeholders::_1));

    m_listener.open().wait();

    m_get_state_client = m_node_handle.serviceClient<adr::get_state>("/adr/get_state");
    m_set_state_client = m_node_handle.serviceClient<adr::set_state>("/adr/set_state");
}

RESTNode::~RESTNode()
{
    m_listener.close().wait();
}

void RESTNode::onGet(web::http::http_request const &request)
{
    auto paths = web::uri::split_path(web::uri::decode(request.relative_uri().path()));

    if (paths[0] == "get_state") {
        adr::get_state get_state_srv;

        if (m_get_state_client.call(get_state_srv)) {
            auto response = web::json::value::object();

            response["state"] = web::json::value::string(get_state_srv.response.state);

            request.reply(web::http::status_codes::OK, response);
        }
        else {
            request.reply(web::http::status_codes::InternalError);
        }
    }
    else {
        request.reply(web::http::status_codes::BadRequest);
    }
}

void RESTNode::onPost(web::http::http_request const &request)
{
    auto paths = web::uri::split_path(web::uri::decode(request.relative_uri().path()));

    auto query = web::uri::split_query(web::uri::decode(request.request_uri().query()));

    if (paths[0] == "set_state") {
        adr::set_state set_state_srv;

        set_state_srv.request.state = query["state"];

        if (m_set_state_client.call(set_state_srv)) {
            request.reply(web::http::status_codes::OK);
        }
        else {
            request.reply(web::http::status_codes::BadRequest);
        }
    }
    else {
        request.reply(web::http::status_codes::BadRequest);
    }
}

int main(int argc, char **argv)
{
    try {
        ros::init(argc, argv, "rest");

        ros::NodeHandle node_handle("~");

        RESTNode rest_node(node_handle);
        
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
