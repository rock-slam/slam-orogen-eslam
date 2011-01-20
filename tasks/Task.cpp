#include "Task.hpp"
#include <algorithm>

using namespace eslam;

Task::Task(std::string const& name)
    : TaskBase(name), 
    aggr( new aggregator::PullStreamAligner() ),
    body_state_valid( false ),
    scan_valid( false )
{
}

template <class T>
bool readPortSample( RTT::InputPort<T> *port, base::Time& ts, T& data)
{
    if( port->read( data ) == RTT::NewData )
    {
	ts = data.time;
	return true;
    }
    return false;
}

void Task::bodystate_callback( base::Time ts, const asguard::BodyState& wbs )
{
    body_state = wbs;
    body_state_valid = true;
}

void Task::scan_callback( base::Time ts, const base::samples::LaserScan& scan )
{
    this->scan = scan;
    scan_valid = true;
}

void Task::orientation_callback( base::Time ts, const base::samples::RigidBodyState& rbs )
{
    if( !body_state_valid )
	return;

    static Eigen::Quaterniond update_orientation;
    static asguard::BodyState update_bodystate; 

    Eigen::Quaterniond orientation(rbs.orientation);

    asguard::BodyState bs( body_state );

    bool updated = false;

    if( useScans && scan_valid )
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

#ifndef DEBUG_VIZ
    if( _pose_distribution.connected() )
#endif
    {
	eslam::PoseDistribution pd;
	pd.time = ts;
	pd.orientation = update_orientation;
	pd.bodyState = update_bodystate;
	std::vector<eslam::PoseEstimator::Particle>& particles( filter->getParticles() );
	std::copy( 
		particles.begin(), 
		particles.end(), 
		std::back_inserter(pd.particles) );

	if( _pose_distribution.connected() )
	    _pose_distribution.write( pd );

#ifdef DEBUG_VIZ
	viz.widget->setPoseDistribution( pd );
	viz.widget->setReferencePose( centroid, bs );
	const int inspect_particle_idx = viz.widget->getInspectedParticleIndex();

	if( updated )
	{
	    typedef eslam::PoseEstimator::Particle Particle;
	    std::vector<eslam::PoseEstimator::Particle>::iterator el =
		std::max_element( particles.begin(), particles.end(), 
			boost::bind( &Particle::weight, _1 ) < boost::bind( &Particle::weight, _2 )  );
	    envire::MultiLevelSurfaceGrid* grid;
	    //const size_t best_particle_idx = el - particles.begin(); 
	    if( inspect_particle_idx >= 0 )
		grid = particles[inspect_particle_idx].grid.get();
	    else
		grid = el->grid.get();

	    if( grid )
	    {
		std::vector<envire::MultiLevelSurfaceGrid*> vizGrids = vizEnv->getItems<envire::MultiLevelSurfaceGrid>();
		if( !vizGrids.empty() )
		{
		    // copy the selected grid to be visualized
		    // this is actually quite inefficient, but works for now
		    envire::MultiLevelSurfaceGrid *vizGrid = vizGrids.front();
		    vizGrid->operator=( *grid );
		    vizEnv->itemModified( vizGrid );
		}
		else
		{
		    // create a new mls map in the viz environment
		    envire::MultiLevelSurfaceGrid *vizGrid = grid->clone();
		    vizEnv->attachItem( vizGrid );
		    envire::FrameNode *fn = new envire::FrameNode( grid->getFrameNode()->getTransform() );
		    vizEnv->addChild( vizEnv->getRootNode(), fn );
		    vizGrid->setFrameNode( fn );
		}
	    }
	    else
		std::cerr << "WARN: could not find grid with largest weight." << std::endl;
	}
#endif
    }
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    // initialize the filter object with the current configuration
    filter = boost::shared_ptr<eslam::EmbodiedSlamFilter>( new eslam::EmbodiedSlamFilter( 
		_asguard_config.get(),
		_odometry_config.get(), 
		_eslam_config.get() ) ); 

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
    viz.start();

    vizEnv = boost::shared_ptr<envire::Environment>( new envire::Environment() );
    viz.widget->setEnvironment( vizEnv.get() );
#endif

    // init the filter
    base::Pose pose( _start_pose.value().position, _start_pose.value().orientation );
    filter->init( env.get(), pose, false );

    std::cerr << "initialized" << std::endl;

    // setup the aggregator with the timeout value provided by the module
    aggr->setTimeout( base::Time::fromSeconds( _max_delay.value() ) );

    // a priority value of 1 will make sure, the orientation callback is call second
    // for a timestamp with the same value 
    orientation_idx = aggr->registerStream<base::samples::RigidBodyState>(
	   boost::bind( readPortSample<base::samples::RigidBodyState>, &_orientation_samples, _1, _2 ),
	   boost::bind( &Task::orientation_callback, this, _1, _2 ), -1,
	   base::Time::fromSeconds( _orientation_period.value() ), 1 );

    bodystate_idx = aggr->registerStream<asguard::BodyState>(
	   boost::bind( readPortSample<asguard::BodyState>, &_bodystate_samples, _1, _2 ),
	   boost::bind( &Task::bodystate_callback, this, _1, _2 ), -1, 
	   base::Time::fromSeconds( _bodystate_period.value() ) );

    scan_idx = aggr->registerStream<base::samples::LaserScan>(
	   boost::bind( readPortSample<base::samples::LaserScan>, &_scan_samples, _1, _2 ),
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
    while(aggr->pull()) 
	while(aggr->step());

    // only write output if the aggregator actually had some data
    // this is slightly implicit, and could be made more explicit in
    // the aggregator
    if( aggr->getLatestTime() > base::Time() )
	_streamaligner_status.write( aggr->getStatus() );
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

