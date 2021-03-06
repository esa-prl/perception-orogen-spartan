/* Generated from orogen/lib/orogen/templates/tasks/MappingTask.cpp */

#include "MappingTask.hpp"

using namespace spartan;
using namespace Eigen;

MappingTask::MappingTask(std::string const& name)
    : MappingTaskBase(name)
{
    frame_num_left = 0;
    frame_num_right = 0;
}

MappingTask::~MappingTask()
{
}

// Simple logging utility for seeing frames right before SPARTAN
// core receives them, i.e. after any calibration/undistortion has
// been applied. To view, find the appropriate log folder and run:
// $ rock-replay <logfile_name>.
void MappingTask::logProcessedFrames()
{
    base::samples::frame::Frame
        *proc_frame_left = mpme->getImgFrame(0),
        *proc_frame_right = mpme->getImgFrame(1);

    _processed_frame_left.write(proc_frame_left);
    _processed_frame_right.write(proc_frame_right);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MappingTask.hpp for more detailed
// documentation about them.

bool MappingTask::configureHook()
{
    if (! MappingTaskBase::configureHook())
        return false;

    mpil = new ImageLoader(_calibration_confs.get());

    CalibInfo ci = mpil->getCalibInfo();
    Affine3d lcam2body_tf;
    base::Time ts;
    if (!_left_camera_viso22body.get(ts, lcam2body_tf, true)) {
        std::cerr << "Unable to retrieve transform -> defaulting to IDENTITY!!!" << std::endl;
        lcam2body_tf = Affine3d::Identity();
    }
    mpme = new MappingExecutor(_mapping_confs.get(),
            ci, lcam2body_tf);

    return mpme != NULL && mpil != NULL;
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

    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> incoming;
    base::samples::frame::Frame *incFrame;

    bool found_fresh = false;
    // WARNING: When checking for _input_port.read(...) it makes
    // sense to compare to RTT::NewData, a constant value that
    // signifies that fresh data is available to be read, which
    // the application has not consumed in the past, otherwise
    // old data will continually be read each time.
    if (_img_in_left.read(incoming) == RTT::NewData) {
        std::cout << "MAPPER RECEIVED LEFT: " << ++frame_num_left << std::endl;
        incFrame = new Frame(*incoming);
        mpil->newFrame(LEFT_CAMERA_FEED, incFrame);
        found_fresh = true;
        std::cout << "MAPPER DELETING LEFT" << std::endl;
        delete incFrame;
        std::cout << "MAPPER DELETED LEFT" << std::endl;
    }
    if (_img_in_right.read(incoming) == RTT::NewData) {
        std::cout << "MAPPER RECEIVED RIGHT: " << ++frame_num_right << std::endl;
        incFrame = new Frame(*incoming);
        mpil->newFrame(RIGHT_CAMERA_FEED, incFrame);
        found_fresh = true;
        std::cout << "MAPPER DELETING RIGHT" << std::endl;
        delete incFrame;
        std::cout << "MAPPER DELETED RIGHT" << std::endl;
    }
    // Even though the task is port triggerred, this needs to be
    // here to accomodate the fact that updateHook is called when
    // the process begins, so the first call will have no data
    // available and should not try using the empty init values.
    if (! found_fresh) {
        if (_world2body.read(w2b_tf) == RTT::NewData) {
            std::cout << "Mapper received transform" << std::endl;
        } else {
            fprintf(stderr, "Unreachable, assuming we're at startup.\n");
            return;
        }
    }

    // Request frame pair from ImageLoader
    FramePair *fp = mpil->getFramePair();
    if (fp == NULL) {
        //std::cout
        //    << "Pair unavailable, returning."
        //    << std::endl;
        return;
    }

    // Initiate map generation from MappingExecutor
    // Careful: w2b_tf is still a RigidBodyState, but it allows us to
    // get a const Eigen::Affine3d by calling getTransfrom(), which
    // is what generateMap expects to see
    if (frame_num_left == frame_num_right) {
        mpme->generateMap(*fp, w2b_tf.getTransform());
    }

    // Uncomment this line to log frames right before they enter
    // the SPARTAN core algorithm
    logProcessedFrames();
    _map_out.write(mpme->getTransportable());

    return;
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

    delete mpil;
    delete mpme;
}
