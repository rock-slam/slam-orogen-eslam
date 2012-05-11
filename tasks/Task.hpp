#ifndef ESLAM_TASK_TASK_HPP
#define ESLAM_TASK_TASK_HPP

#include "eslam/TaskBase.hpp"

#include <boost/shared_ptr.hpp>
#include <eslam/EmbodiedSlamFilter.hpp>
#include <asguard/Odometry.hpp>
#include <envire/Core.hpp>
#include <vizkit/MapVizEventFilter.hpp>

#include <aggregator/PullStreamAligner.hpp>

#include <vizkit/EslamWidget.hpp>
#include <vizkit/QtThreadedWidget.hpp>
#include <envire/Orocos.hpp>

namespace eslam {

    class Task : public TaskBase
    {
	friend class TaskBase;

    protected:
	boost::shared_ptr<eslam::EmbodiedSlamFilter> filter;
	boost::shared_ptr<envire::Environment> env;

	// debug temporaries
	Eigen::Quaterniond update_orientation;
	odometry::BodyContactState update_bodystate; 
	// derived configuration variable 
	bool doMapping;
	odometry::BodyContactState lastContactState; 

	envire::OrocosEmitter* orocosEmitter;
	vizkit::MapVizEventFilter* mapFilter;
	base::Time lastEnvireDataUpdate;

	QtThreadedWidget<vizkit::EslamWidget> viz;

        virtual ::base::samples::RigidBodyState cloneMap();
	// transformer callbacks
        virtual void bodystate_samplesTransformerCallback(const base::Time &ts, const ::odometry::BodyContactState &bodystate_samples_sample);
        virtual void distance_framesTransformerCallback(const base::Time &ts, const ::base::samples::DistanceImage &distance_frames_sample);
        virtual void scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample);
        virtual void terrain_classification_framesTransformerCallback(const base::Time &ts, const ::base::samples::frame::Frame &terrain_classification_frames_sample);
        virtual void terrain_classification_wheelTransformerCallback(const base::Time &ts, const ::terrain_estimator::TerrainClassification &terrain_classification_wheel_sample);
        virtual void orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample);

	// store last terrain classifications 
	std::vector<terrain_estimator::TerrainClassification> terrainClassificationWheel;
	base::samples::frame::Frame terrainClassificationFrame;

	/// update mainly debug information and the state of the filter
	void updateFilterInfo( const base::Time& ts, const odometry::BodyContactState& bs, base::Affine3d& centroid, bool updated );

    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "eslam::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

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

