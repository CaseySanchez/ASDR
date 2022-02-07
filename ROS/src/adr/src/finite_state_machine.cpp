#include "finite_state_machine.hpp"

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

void Automatic::entryGuard(GuardControl &control) noexcept 
{    
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

        control.changeTo<Idle>();
    }
}

void Automatic::enter(Control &control) noexcept
{
    ROS_INFO("Automatic state entered.");
}

void Automatic::update(FullControl &control) noexcept 
{
}

void Automatic::exitGuard(GuardControl &control) noexcept
{
    try {
        m_ros_launch_manager.stop(m_rtabmap_pid, SIGINT);
        m_ros_launch_manager.stop(m_realsense_pid, SIGINT);
    }
    catch (std::exception const &exception) {
        ROS_WARN("%s", exception.what());

        control.changeTo<Idle>();
    }
}

void Automatic::exit(Control &control) noexcept
{
    ROS_INFO("Automatic state exited.");
}

void Delay::enter(Control &control) noexcept 
{
    ROS_INFO("Delay state entered.");
    
    m_start = ros::Time::now();
    m_delay = ros::Duration(10.0);
}

void Delay::update(FullControl &control) noexcept 
{
    if (ros::Time::now() > m_start + m_delay) {
        control.changeTo<Discover>();
    }
}

void Delay::exit(Control &control) noexcept
{
    ROS_INFO("Delay state exited.");
}

void Discover::entryGuard(GuardControl &control) noexcept 
{
    m_set_mode_mapping_client = control.context().m_node_handle.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_mapping");

    std_srvs::Empty empty_srv;

    if (!m_set_mode_mapping_client.call(empty_srv)) {
        ROS_WARN("Failed to set RTABMAP mode to mapping.");

        control.changeTo<Idle>();
    }
}

void Discover::enter(Control &control) noexcept
{
    ROS_INFO("Discover state entered.");
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

void Disinfect::entryGuard(GuardControl &control) noexcept 
{
    m_set_mode_localization_client = control.context().m_node_handle.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");
    m_set_uvc_light_client = control.context().m_node_handle.serviceClient<uvc_light::set_uvc_light>("/dev/ttyUSB0/set_uvc_light");

    std_srvs::Empty set_mode_localization_srv;

    if (!m_set_mode_localization_client.call(set_mode_localization_srv)) {
        ROS_WARN("Failed to set RTABMAP mode to localization.");

        control.changeTo<Idle>();
    }

    uvc_light::set_uvc_light set_uvc_light_srv;

    set_uvc_light_srv.request.state = true;

    if (!m_set_uvc_light_client.call(set_uvc_light_srv)) {
        ROS_WARN("Failed to turn UVC light on.");

        control.changeTo<Idle>();
    }
}

void Disinfect::enter(Control &control) noexcept
{
    ROS_INFO("Disinfect state entered.");
}

void Disinfect::update(FullControl &control) noexcept 
{
}

void Disinfect::exitGuard(GuardControl &control) noexcept
{
    uvc_light::set_uvc_light set_uvc_light_srv;

    set_uvc_light_srv.request.state = false;

    if (!m_set_uvc_light_client.call(set_uvc_light_srv)) {
        ROS_WARN("Failed to turn UVC light off.");

        control.changeTo<Idle>();
    }
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
    control.changeTo<Navigate>();
}

void Plan::exit(Control &control) noexcept
{
    ROS_INFO("Plan state exited.");
}

void Navigate::enter(Control &control) noexcept 
{
    ROS_INFO("Navigate state entered.");
}

void Navigate::update(FullControl &control) noexcept 
{
    control.changeTo<Idle>();
}

void Navigate::exit(Control &control) noexcept
{
    ROS_INFO("Navigate state exited.");
}
