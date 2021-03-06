/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SPARTAN_TASK_TASK_HPP
#define SPARTAN_TASK_TASK_HPP

#include "spartan/TaskBase.hpp"

#include <spartan/Config.hpp>
#include <spartan/CalibInfo.hpp>
#include <spartan/ImageLoader.hpp>
#include <spartan/OdometryExecutor.hpp>

#include <base/samples/RigidBodyState.hpp>

#include <vector>
#include <Eigen/Dense>
#include <ctime>

#include <thread>
#include <chrono>


using namespace Eigen;

namespace spartan{

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * This is the main VO task:
{left frame, right frame} => {cumulative pose with respect to origin}
additional outputs are available, i.e. "processed_frame_*", to help
with visualizing what frames are fed to the core VO algorithm after
any preprocessing steps. These frames correspond to the initial images
after undistortion, rectification and rescaling, see ImageLoader for
the implementation details.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','spartan::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        std::vector<double> centeredPose;
        ImageLoader *mpil;
        OdometryExecutor *mpoe;
        uint8_t camera_feed_code;
        void logProcessedFrames();
        base::samples::RigidBodyState RBS_new, RBS_old, RBS_delta;
        Eigen::Affine3d T_new, T_old, T_delta;
        Affine3d lcam2body_tf;
        double vo_computation_time, period_des_s, wait_des, IFD_des;
        bool first_vo_computed, start;
        //Affine3d lcam2body_tf;
        base::Time start_time, current_time;
        base::commands::Motion2D motion_command;

    protected:
        //virtual void mast_to_ptu_inTransformerCallback(base::Time const& timestamp, base::samples::RigidBodyState const& sample);
        double setNewVOPeriod(base::commands::Motion2D motion_command);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "spartan::Task");

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

