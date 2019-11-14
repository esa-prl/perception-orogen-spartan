/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace spartan;
using namespace Eigen;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    centeredPose = vector<double>(NUM_QRTPARAMS);
}

Task::~Task()
{
}

// Simple logging utility for seeing frames right before SPARTAN
// core receives them, i.e. after any calibration/undistortion has
// been applied. To view, find the appropriate log folder and run:
// $ rock-replay <logfile_name>.
void Task::logProcessedFrames()
{
    base::samples::frame::Frame
        proc_frame_left = mpoe->getImgFrame(0),
        proc_frame_right = mpoe->getImgFrame(1);

    _processed_frame_left.write(proc_frame_left);
    _processed_frame_right.write(proc_frame_right);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    mpil = new ImageLoader(_calibration_confs.get());

    CalibInfo ci = mpil->getCalibInfo();
    Affine3d lcam2body_tf;
    RBS_old.setTransform( Affine3d::Identity() );
    base::Time ts;
    if (!_left_camera_viso22body.get(ts, lcam2body_tf, true)) {
        std::cerr << "Unable to retrieve transform!" << std::endl;
        exit(1);
    }
    mpoe = new OdometryExecutor(_calibration_confs.get(),
            ci, lcam2body_tf);

    return mpoe != NULL && mpil != NULL;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    base::samples::frame::Frame incoming =
        base::samples::frame::Frame();

    bool found_fresh = false;
    // WARNING: When checking for _input_port.read(...) it makes
    // sense to compare to RTT::NewData, a constant value that
    // signifies that fresh data is available to be read, which
    // the application has not consumed in the past, otherwise
    // old data will continually be read each time.
    if (_img_in_left.read(incoming) == RTT::NewData) {
        //std::cout << "SVO RECEIVED LEFT" << std::endl;
        mpil->newFrame(LEFT_CAMERA_FEED, &incoming);
        found_fresh = true;
    } else if (_img_in_right.read(incoming) == RTT::NewData) {
        //std::cout << "SVO RECEIVED RIGHT" << std::endl;
        mpil->newFrame(RIGHT_CAMERA_FEED, &incoming);
        found_fresh = true;
    }
    // Even though the task is port triggerred, this needs to be
    // here to accomodate the fact that updateHook is called when
    // the process begins, so the first call will have no data
    // available and should not try using the empty init values.
    if (! found_fresh) {
        fprintf(stderr, "Unreachable, assuming we're at startup.\n");
        return;
    }

    // Request frame pair from ImageLoader
    FramePair *fp = mpil->getFramePair();
    if (fp == NULL) {
        //std::cout
        //    << "Pair unavailable, returning."
        //    << std::endl;
        return;
    }

    // Request next pose from OdometryExecutor
    std::clock_t begin = std::clock();
    mpoe->nextPose(*fp, centeredPose);
    std::clock_t end = std::clock();
    vo_computation_time = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
    _computation_time.write(vo_computation_time);

    // Uncomment this line to log frames right before they enter
    // the SPARTAN core algorithm
    logProcessedFrames();

    printf("\nCentered pose updated:\n");
    for (float elem : centeredPose) {
        printf("%f\n", elem);
    }

    // Write the output pose to the output port, formatted
    // as a RigidBodyState
    RBS_new = mpoe->getRBS();
    RBS_new.time = incoming.time;
    _vo_out.write(RBS_new);

    // Optionally do extra stuff with the updated vo_state struct
    //vostate *temp = mpoe->getVisOdomState();
    
    // Compute and output the delta_pose (T_new = T_delta*T_old) converted to RigidBodyState
    T_new = RBS_new.getTransform();
    T_old = RBS_old.getTransform();
    T_delta = T_old.inverse()*T_new;
    RBS_delta.setTransform(T_delta);
    RBS_delta.time = incoming.time;
    _delta_vo_out.write(RBS_delta);
    
    // update RBS_old
    RBS_old = RBS_new;
    
    return;
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    delete mpil;
    delete mpoe;
}
