/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MappingTask.hpp"

using namespace spartan;

MappingTask::MappingTask(std::string const& name)
    : MappingTaskBase(name)
{
}

MappingTask::~MappingTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MappingTask.hpp for more detailed
// documentation about them.

bool MappingTask::configureHook()
{
    if (! MappingTaskBase::configureHook())
        return false;
    return true;
}
bool MappingTask::startHook()
{
    if (! MappingTaskBase::startHook())
        return false;
    return true;
}
void MappingTask::updateHook()
{
    MappingTaskBase::updateHook();
}
void MappingTask::errorHook()
{
    MappingTaskBase::errorHook();
}
void MappingTask::stopHook()
{
    MappingTaskBase::stopHook();
}
void MappingTask::cleanupHook()
{
    MappingTaskBase::cleanupHook();
}
