#include "state_machine.hpp"

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
}

void Automatic::update(FullControl &control) noexcept 
{
}

void Automatic::exit(Control &control) noexcept
{
    ROS_INFO("Automatic state exited.");
}