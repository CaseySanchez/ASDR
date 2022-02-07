#pragma once

#include <stdexcept>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "std_srvs/Empty.h"

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
struct LightOn;
struct Navigate;
struct LightOff;

using M = hfsm2::MachineT<hfsm2::Config::ContextT<Context>>;
using FSM = M::PeerRoot<
                Idle,
                Manual,
                M::Composite<Automatic,
                    Delay,
                    M::Composite<Discover,
                        Observe,
                        Explore
                    >,
                    M::Composite<Disinfect,
                        Plan,
                        LightOn,
                        Navigate,
                        LightOff
                    >
                >
            >;

struct Idle : public FSM::State 
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Manual : public FSM::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Automatic : public FSM::State
{
    ROSLaunchManager m_ros_launch_manager;

    pid_t m_realsense_pid;
    pid_t m_rtabmap_pid;

	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Delay : public FSM::State
{
    ros::Time m_start;

	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Discover : public FSM::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Observe : public FSM::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Explore : public FSM::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Disinfect : public FSM::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Plan : public FSM::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct LightOn : public FSM::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct Navigate : public FSM::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};

struct LightOff : public FSM::State
{
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};