#include "state_machine.hpp"

Context::Context(ros::NodeHandle const &node_handle) : m_node_handle(node_handle)
{
}

void Idle::enter(Control &control) noexcept 
{
    ROS_INFO("Idle state entered.");
}

void Idle::update(FullControl &control) noexcept 
{
}

void Idle::exit(Control &control) noexcept
{
    ROS_INFO("Idle state exited.");
}

void Manual::enter(Control &control) noexcept 
{
    ROS_INFO("Manual state entered.");
}

void Manual::update(FullControl &control) noexcept 
{
}

void Manual::exit(Control &control) noexcept
{
    ROS_INFO("Manual state exited.");
}

void Automatic::enter(Control &control) noexcept 
{
    ROS_INFO("Automatic state entered.");
    
    try {
        m_realsense_pid = m_ros_launch_manager.start(
            "realsense2_camera", "rs_camera.launch",
            "align_depth:=true");

        m_rtabmap_pid = m_ros_launch_manager.start(
            "rtabmap_ros", "rtabmap.launch", 
            "rtabmap_args:=\"--delete_db_on_start\"", 
            "depth_topic:=/camera/aligned_depth_to_color/image_raw", 
            "rgb_topic:=/camera/color/image_raw",
            "camera_info_topic:=/camera/color/camera_info",
            "approx_sync:=false");
    }
    catch (std::exception const &exception) {
        ROS_WARN("%s", exception.what());
    }

    //ros::Duration(5.0).sleep();
}

void Automatic::update(FullControl &control) noexcept 
{
}

void Automatic::exit(Control &control) noexcept
{
    ROS_INFO("Automatic state exited.");

    try {
        m_ros_launch_manager.stop(m_rtabmap_pid, SIGINT);
        m_ros_launch_manager.stop(m_realsense_pid, SIGINT);
    }
    catch (std::exception const &exception) {
        ROS_WARN("%s", exception.what());
    }

    //ros::Duration(5.0).sleep();
}

void Discover::enter(Control &control) noexcept 
{
    ROS_INFO("Discover state entered.");

    /*ros::ServiceClient set_mode_mapping_client = control.context().m_node_handle.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_mapping");

    std_srvs::Empty empty_srv;

    set_mode_mapping_client.call(empty_srv);*/
}

void Discover::update(FullControl &control) noexcept 
{
}

void Discover::exit(Control &control) noexcept
{
    ROS_INFO("Discover state exited.");
}

void Observe::enter(Control &control) noexcept 
{
    ROS_INFO("Observe state entered.");
}

void Observe::update(FullControl &control) noexcept 
{
    control.changeTo<Explore>();
}

void Observe::exit(Control &control) noexcept
{
    ROS_INFO("Observe state exited.");
}

void Explore::enter(Control &control) noexcept 
{
    ROS_INFO("Explore state entered.");
}

void Explore::update(FullControl &control) noexcept 
{
    control.changeTo<Disinfect>();
}

void Explore::exit(Control &control) noexcept
{
    ROS_INFO("Explore state exited.");
}

void Disinfect::enter(Control &control) noexcept 
{
    ROS_INFO("Disinfect state entered.");

    /*ros::ServiceClient set_mode_localization_client = control.context().m_node_handle.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");

    std_srvs::Empty empty_srv;

    set_mode_localization_client.call(empty_srv);*/
}

void Disinfect::update(FullControl &control) noexcept 
{
}

void Disinfect::exit(Control &control) noexcept
{
    ROS_INFO("Disinfect state exited.");
}

void Plan::enter(Control &control) noexcept 
{
    ROS_INFO("Plan state entered.");
}

void Plan::update(FullControl &control) noexcept 
{
    control.changeTo<LightOn>();
}

void Plan::exit(Control &control) noexcept
{
    ROS_INFO("Plan state exited.");
}

void LightOn::enter(Control &control) noexcept 
{
    ROS_INFO("LightOn state entered.");
}

void LightOn::update(FullControl &control) noexcept 
{
    control.changeTo<Navigate>();
}

void LightOn::exit(Control &control) noexcept
{
    ROS_INFO("LightOn state exited.");
}

void Navigate::enter(Control &control) noexcept 
{
    ROS_INFO("Navigate state entered.");
}

void Navigate::update(FullControl &control) noexcept 
{
    control.changeTo<LightOff>();
}

void Navigate::exit(Control &control) noexcept
{
    ROS_INFO("Navigate state exited.");
}

void LightOff::enter(Control &control) noexcept 
{
    ROS_INFO("LightOff state entered.");

    start = ros::Time::now();
}

void LightOff::update(FullControl &control) noexcept 
{
    if (ros::Time::now() > start + ros::Duration(10.0)) {
        control.changeTo<Idle>();
    }
}

void LightOff::exit(Control &control) noexcept
{
    ROS_INFO("LightOff state exited.");
}