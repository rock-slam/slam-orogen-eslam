#include "Task.hpp"

#include <rtt/NonPeriodicActivity.hpp>

#include <ParticleWrapper.hpp>

using namespace eslam;

RTT::NonPeriodicActivity* Task::getNonPeriodicActivity()
{ return dynamic_cast< RTT::NonPeriodicActivity* >(getActivity().get()); }

Task::Task(std::string const& name)
    : TaskBase(name), config( new asguard::Configuration() ), 
    filter( new eslam::EmbodiedSlamFilter(*config) ),
    aggr( new aggregator::StreamAligner() )
{
}

void Task::bodystate_callback( base::Time ts, const wrappers::BodyState& wbs )
{
    body_state = wbs;
}

void Task::scan_callback( base::Time ts, const base::samples::LaserScan& scan )
{
    this->scan = scan;
}

void Task::orientation_callback( base::Time ts, const wrappers::samples::RigidBodyState& rbs )
{
    static Eigen::Quaterniond update_orientation;
    static asguard::BodyState update_bodystate; 

    Eigen::Quaterniond orientation(rbs.orientation);

    asguard::BodyState bs( body_state );

    bool updated = false;

    if( useScans )
	updated = filter->update( bs, orientation, scan );
    else 
	updated = filter->update( bs, orientation );

    // record bodystate and orientation when an update 
    // has occured. this is mainly for visualizing purposes,
    // so to capture the updated state.
    if( updated )
    {
	update_orientation = orientation;
	update_bodystate = bs;
    }

    base::Pose centroid = filter->getCentroid();

    base::samples::RigidBodyState res_rbs;
    res_rbs.setPose( centroid );

    _pose_samples.write( res_rbs );

    if( _pose_distribution.connected() )
    {
	wrappers::PoseDistribution pd;
	pd.time = ts;
	pd.orientation = update_orientation;
	pd.body_state = update_bodystate;
	const std::vector<wrappers::PoseParticle::particle>& particles( filter->getParticles() );
	std::copy( 
		particles.begin(), 
		particles.end(), 
		std::back_inserter(pd.particles) );

	_pose_distribution.write( pd );
#ifdef DEBUG_VIZ
	viz.widget->setPoseDistribution( pd );
	viz.widget->setReferencePose( centroid, bs );
#endif
    }
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    // load an environment if path is specified
    if( !_environment_path.value().empty() )
    {
	envire::Serialization so;
	env = boost::shared_ptr<envire::Environment>( so.unserialize( _environment_path.value() ) );
	useScans = false;
    }
    else
    {
	env = boost::shared_ptr<envire::Environment>( new envire::Environment() );
	useScans = true;
    }

#ifdef DEBUG_VIZ
    while( !viz.isInitialized() )
	usleep( 200 );
	
    viz.widget->setEnvironment( env.get() );
#endif

    // init the filter
    base::Pose pose( _start_pose.value().position, _start_pose.value().orientation );
    filter->init( env.get(), pose );

    std::cerr << "initialized" << std::endl;

    // setup the aggregator with the timeout value provided by the module
    aggr->setTimeout( base::Time::fromSeconds( _max_delay.value() ) );

    // a priority value of 1 will make sure, the orientation callback is call second
    // for a timestamp with the same value 
    orientation_idx = aggr->registerStream<wrappers::samples::RigidBodyState>(
	   boost::bind( &Task::orientation_callback, this, _1, _2 ), -1,
	   base::Time::fromSeconds( _orientation_period.value() ), 1 );

    bodystate_idx = aggr->registerStream<wrappers::BodyState>(
	   boost::bind( &Task::bodystate_callback, this, _1, _2 ), -1, 
	   base::Time::fromSeconds( _bodystate_period.value() ) );

    scan_idx = aggr->registerStream<base::samples::LaserScan>(
	   boost::bind( &Task::scan_callback, this, _1, _2 ), -1, 
	   base::Time::fromSeconds( _scan_period.value() ) );

    return true;
}

bool Task::startHook()
{
    return true;
}

void Task::updateHook()
{
    static int idx = 0;

    if( (idx++ % 100) == 0 )
	std::cerr << "updateHook " << idx << "\r";

    wrappers::samples::RigidBodyState orientation_sample;
    while( _orientation_samples.read( orientation_sample ) )
    {
	aggr->push( orientation_idx, orientation_sample.time, orientation_sample );	
    }
    
    wrappers::BodyState bodystate_sample;
    while( _bodystate_samples.read( bodystate_sample ) )
    {
	aggr->push( bodystate_idx, bodystate_sample.time, bodystate_sample );	
    }

    base::samples::LaserScan scan_sample;
    while( _scan_samples.read( scan_sample ) )
    {
	aggr->push( scan_idx, scan_sample.time, scan_sample );	
    }

    while(aggr->step());
}

// void Task::errorHook()
// {
// }

void Task::stopHook()
{
    // write environment, if path is given
    if( !_environment_debug_path.value().empty() )
    {
	envire::Serialization so;
	so.serialize(env.get(), _environment_debug_path.value() );
    }
}

// void Task::cleanupHook()
// {
// }

