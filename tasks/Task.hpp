#ifndef ESLAM_TASK_TASK_HPP
#define ESLAM_TASK_TASK_HPP

#include "eslam/TaskBase.hpp"

#include <boost/shared_ptr.hpp>
#include <asguard/EmbodiedSlamFilter.hpp>
#include <asguard/Odometry.hpp>
#include <envire/Core.hpp>

#include <StreamAligner.hpp>

namespace RTT
{
    class NonPeriodicActivity;
}


namespace eslam {
    class Task : public TaskBase
    {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	friend class TaskBase;

    protected:
	boost::shared_ptr<asguard::Configuration> config;
	boost::shared_ptr<eslam::EmbodiedSlamFilter> filter;
	boost::shared_ptr<envire::Environment> env;

	boost::shared_ptr<aggregator::StreamAligner> aggr;

	int bodystate_idx;
	int orientation_idx;
    
	void bodystate_callback( base::Time ts, const wrappers::BodyState& body_state );
	void orientation_callback( base::Time ts, const wrappers::samples::RigidBodyState& orientation );

	Eigen::Quaterniond orientation;

    public:
        Task(std::string const& name = "eslam::Task");

        RTT::NonPeriodicActivity* getNonPeriodicActivity();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
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
         * when the hook should be called. See README.txt for different
         * triggering options.
         *
         * The warning(), error() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeWarning, RunTimeError and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recovered()
         * allows you to go back into the Running state.  In the second case,
         * the errorHook() will be called instead of updateHook() and in the
         * third case the component is stopped and resetError() needs to be
         * called before starting it again.
         *
         */
        void updateHook();
        

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recovered() to go back in the Runtime state.
         */
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        // void cleanupHook();
    };
}

#endif

