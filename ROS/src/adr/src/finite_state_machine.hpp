#pragma once

#include <stdexcept>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "std_srvs/Empty.h"
#include "uvc_light/set_uvc_light.h"

#include "ros_launch_manager.hpp"

#include "hfsm2/machine.hpp"

struct Context 
{
    Context(ros::NodeHandle const &node_handle);

    ros::NodeHandle m_node_handle;
};

struct Idle;
struct Manual;
struct Automatic;
struct Delay;
struct Discover;
struct Observe;
struct Explore;
struct Disinfect;
struct Plan;
struct Navigate;

using MachineT = hfsm2::MachineT<hfsm2::Config::ContextT<Context>>;
using FiniteStateMachineT = MachineT::PeerRoot<
                Idle,
                Manual,
                MachineT::Composite<Automatic,
                    Delay,
                    MachineT::Composite<Discover,
                        Observe,
                        Explore
                    >,
                    MachineT::Composite<Disinfect,
                        Plan,
                        Navigate
                    >
                >
            >;

struct Idle : public FiniteStateMachineT::State 
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Manual : public FiniteStateMachineT::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Automatic : public FiniteStateMachineT::State
{
    ROSLaunchManager m_ros_launch_manager;

    pid_t m_realsense_pid;
    pid_t m_rtabmap_pid;

	void entryGuard(GuardControl &control) noexcept;
    void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exitGuard(GuardControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Delay : public FiniteStateMachineT::State
{
    ros::Time m_start;
    ros::Duration m_delay;

	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Discover : public FiniteStateMachineT::State
{
    ros::ServiceClient m_set_mode_mapping_client;

	void entryGuard(GuardControl &control) noexcept;
    void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Observe : public FiniteStateMachineT::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Explore : public FiniteStateMachineT::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Disinfect : public FiniteStateMachineT::State
{
    ros::ServiceClient m_set_mode_localization_client;
    ros::ServiceClient m_set_uvc_light_client;

	void entryGuard(GuardControl &control) noexcept;
    void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exitGuard(GuardControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Plan : public FiniteStateMachineT::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Navigate : public FiniteStateMachineT::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};
