/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CameraEmulator.hpp"

using namespace spartan;

CameraEmulator::CameraEmulator(std::string const& name)
    : CameraEmulatorBase(name)
{
}

CameraEmulator::~CameraEmulator()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CameraEmulator.hpp for more detailed
// documentation about them.

bool CameraEmulator::configureHook()
{
    if (! CameraEmulatorBase::configureHook())
        return false;
    return true;
}
bool CameraEmulator::startHook()
{
    if (! CameraEmulatorBase::startHook())
        return false;
    return true;
}
void CameraEmulator::updateHook()
{
    CameraEmulatorBase::updateHook();
}
void CameraEmulator::errorHook()
{
    CameraEmulatorBase::errorHook();
}
void CameraEmulator::stopHook()
{
    CameraEmulatorBase::stopHook();
}
void CameraEmulator::cleanupHook()
{
    CameraEmulatorBase::cleanupHook();
}
