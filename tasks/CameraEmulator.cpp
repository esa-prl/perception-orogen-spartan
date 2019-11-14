/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CameraEmulator.hpp"

using namespace spartan;
using namespace Eigen;
using namespace base::samples::frame;

CameraEmulator::CameraEmulator(std::string const& name)
    : CameraEmulatorBase(name)
{
}

CameraEmulator::~CameraEmulator()
{
}

base::samples::frame::Frame * CameraEmulator::readImage(char filename[])
{
    printf("%s\n", filename);

    // Attempt to open a file descriptor and check if it
    // worked. Note the "read" and "binary" flags.
    // WARNING: fp has been CLOSED by the read_JPEG_fp function,
    // don't close it again here!
    FILE *fp;
    if ((fp = fopen(filename, "rb")) == NULL) {
        fprintf(stderr, "Error opening JPG file pointer: %s\n", filename);
        return NULL;
    }

    // CAUTION: Set width or height to 0 to allow jpegio to
    // allocate memory, NEVER set to -1, since that triggers
    // a dimension query, which does not actually load any image
    // data, but only returns size info.
    uint8_t *image_data = NULL;
    int width=0, height=0, bpp=0;
    if(read_JPEG_fp(fp, &image_data, &width, &height, &bpp) != JPEGIO_OK) {
        fprintf(stderr, "Error reading JPG image file '%s'\n", filename);
        return NULL;
    }

    /*
    std::cout << "MIRAGE SENDING: "
        << (int) image_data[0] << ", "
        << (int) image_data[100] << ", "
        << (int) image_data[200] << std::endl;
    */

    // Setting up the return frame requires calling the constructor
    // for the Frame container, which sets modes, sizes etc.
    // Then we need to explicitly set the image data and the time
    // of the frame. Note that we are assuming reading in MODE_RGB
    // @ bpp=3, MODE_GRAYSCALE @ bpp=<other> (bpp = bytes per pixel)
    // but if other formats are involved they may need to be coded
    // in manually (see <ROCK_INSTALL_DIR>/base/types/src/samples/Frame.hpp
    // for a complete list of options)
    base::samples::frame::Frame *ret =
        new base::samples::frame::Frame(
                width,
                height,
                bpp,
                (bpp==3)
                    ? base::samples::frame::MODE_RGB
                    : base::samples::frame::MODE_GRAYSCALE
        );

    ret->setImage(
            image_data,
            ret->getNumberOfBytes()
    );

    ret->time = base::Time::now();

    /* Use this block when debugging to check if file-IO is sane */
    //int response = isjpg(filename);
    //if (response) {
    //    std::cout << "YES: "
    //        << width << ", "
    //        << height << ", "
    //        << bpp
    //        << std::endl;
    //} else {
    //    std::cout << "NO" << std::endl;
    //}

    return ret;
}


/**
 * This method allows for the calculation of a RigidBodyState object,
 * to be written out the corresponding output port (see update hook for
 * usage). Configure this function to suit your needs. For example, you
 * may wish to receive some dynamic transform from the system, or read data
 * from a file, convert it to the proper format and then send it, etc.
 * */
base::samples::RigidBodyState CameraEmulator::generateRBS_out()
{
    // Define the return entity
    base::samples::RigidBodyState result;

    // Define a quaternion for the rotations, stacking them by multiplying
    // angle axis objects with values from the configuration
    Quaterniond rotation = AngleAxisd(mConfig.rotx, Vector3d::UnitX())
        * AngleAxisd(mConfig.roty, Vector3d::UnitY())
        * AngleAxisd(mConfig.rotz, Vector3d::UnitZ());
    
    // Define a translation vector from the configuration
    Vector3d translation(mConfig.trax, mConfig.tray, mConfig.traz);
    
    // Define an identity transform and then apply the above
    // CAUTION: Translate first, then rotate!!!
    Affine3d pose(Affine3d::Identity());
    pose.translate(translation).rotate(rotation);
    
    // Store transform in the RBS to be returned
    result.setTransform(pose);

    return result;
} 



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CameraEmulator.hpp for more detailed
// documentation about them.

bool CameraEmulator::configureHook()
{
    if (! CameraEmulatorBase::configureHook())
        return false;

    // Read config from orogen
    mConfig = _cam_emu_confs.get();

    // Use it to initialize member vars
    frame_counter = mConfig.start_index;

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

    base::samples::frame::Frame *left_frame, *right_frame;

    char leftFileTemplate[MAX_STR_LEN], rightFileTemplate[MAX_STR_LEN];
    char leftFileName[MAX_STR_LEN], rightFileName[MAX_STR_LEN];

    strcpy(leftFileTemplate, mConfig.img_dir);
    strcat(leftFileTemplate, mConfig.left_dir);
    strcat(leftFileTemplate, mConfig.img_file_format);
    sprintf(leftFileName, leftFileTemplate, mConfig.left_mark, frame_counter);

    strcpy(rightFileTemplate, mConfig.img_dir);
    strcat(rightFileTemplate, mConfig.right_dir);
    strcat(rightFileTemplate, mConfig.img_file_format);
    sprintf(rightFileName, rightFileTemplate, mConfig.right_mark, frame_counter);

    // Write out a localization transform (useful for debugging the mapping task)
    _world2body_transform.write(generateRBS_out());

    /* If the simulation sends left frames before right frames */
    if (mConfig.delay > 0) {
        left_frame = readImage(leftFileName);
        if (left_frame != NULL) _img_out_left.write(*left_frame);
        sleep(mConfig.delay);
        right_frame = readImage(rightFileName);
        if (right_frame != NULL) _img_out_right.write(*right_frame);
    }
    /* Else if the right frames come before the left ones */
    else {
        right_frame = readImage(rightFileName);
        if (right_frame != NULL) _img_out_right.write(*right_frame);
        sleep(-(mConfig.delay));
        left_frame = readImage(leftFileName);
        if (left_frame != NULL) _img_out_left.write(*left_frame);
    }
    // Note in the above that we need to check for NULLs, in case
    // the input feeds have been exhausted.

    // Increment the frame counter to iterate to the next files
    frame_counter++;
    return;
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
