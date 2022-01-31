#pragma once

#include <stdexcept>
#include <memory>

#include "ros/ros.h"
#include "ros/console.h"

#include "hfsm2/machine.hpp"

struct Context 
{
	unsigned cycleCount = 0;
};

struct Idle;
struct Manual;
struct Automatic;

using FSM = hfsm2::MachineT<hfsm2::Config::ContextT<Context>>::PeerRoot<
                Idle,
                Manual,
                Automatic
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
	void enter(Control &control) noexcept;
	void update(FullControl &control) noexcept;
    void exit(Control &control) noexcept;
};
