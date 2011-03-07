#include "Task.hpp"
#include <algorithm>

#include <envire/tools/GraphViz.hpp>

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
    res_rbs.time = ts;
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
	if( _debug_viz.value() )
	{
	    viz.getWidget()->setPoseDistribution( pd );
	    viz.getWidget()->setBodyState( bs );
	    viz.getWidget()->setCentroidPose( centroid );
	    base::samples::RigidBodyState ref_pose;
	    while( _reference_pose.read( ref_pose ) == RTT::NewData )
		viz.getWidget()->setReferencePose( ref_pose.getPose() );
	    const int inspect_particle_idx = viz.getWidget()->getInspectedParticleIndex();

	    if( updated )
	    {
		typedef eslam::PoseEstimator::Particle Particle;
		std::vector<eslam::PoseEstimator::Particle>::iterator el =
		    std::max_element( particles.begin(), particles.end(), 
			    boost::bind( &Particle::weight, _1 ) < boost::bind( &Particle::weight, _2 )  );
		envire::MLSMap* map;
		//const size_t best_particle_idx = el - particles.begin(); 
		if( inspect_particle_idx >= 0 )
		    map = particles[inspect_particle_idx].grid.getMap();
		else
		    map = el->grid.getMap();

		if( map && !viz.getWidget()->isDirty() && vizEnv != env )
		{
		    // remove all previous maps
		    std::vector<envire::MultiLevelSurfaceGrid*> vizGrids = vizEnv->getItems<envire::MultiLevelSurfaceGrid>();
		    for( std::vector<envire::MultiLevelSurfaceGrid*>::iterator it = vizGrids.begin();
			    it != vizGrids.end(); it++ )
		    {
			envire::FrameNode *fn = (*it)->getFrameNode();
			while( fn && fn->isAttached() && (fn != vizEnv->getRootNode()) )
			{
			    envire::FrameNode* parent = fn->getParent();
			    vizEnv->detachItem( fn );
			    fn = parent;
			}
			vizEnv->detachItem( *it );
		    }

		    // replicate framenode
		    envire::FrameNode* fn = new envire::FrameNode( 
			    env->relativeTransform( map->getFrameNode(), map->getEnvironment()->getRootNode() ) );
		    vizEnv->addChild( vizEnv->getRootNode(), fn );

		    // copy grids from the best map to the visualization environment
		    for( std::vector<envire::MultiLevelSurfaceGrid::Ptr>::iterator it = map->grids.begin();
			    it != map->grids.end(); it++ )
		    {
			// create a new mls map in the viz environment
			envire::MultiLevelSurfaceGrid *vizGrid = (*it)->clone();
			envire::FrameNode *mapNode = (*it)->getFrameNode()->clone();
			vizEnv->addChild( fn, mapNode );
			vizEnv->setFrameNode( vizGrid, mapNode );
			fn = mapNode;
		    }

		    viz.getWidget()->setDirty();
		    /*
		       envire::GraphViz viz;
		       viz.writeToFile( vizEnv.get(), "/tmp/vizEnv.dot" );
		       viz.writeToFile( env.get(), "/tmp/env.dot" );
		       */
		}
	    }
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
    bool useShared = false;
    if( !_environment_path.value().empty() )
    {
	std::cout << "loading environment: " << _environment_path.value() << std::endl;
	envire::Serialization so;
	env = boost::shared_ptr<envire::Environment>( so.unserialize( _environment_path.value() ) );
	useScans = false;
	useShared = true;
    }
    else
    {
	env = boost::shared_ptr<envire::Environment>( new envire::Environment() );
	useScans = true;
    }

#ifdef DEBUG_VIZ
    if( _debug_viz.value() )
    {
	viz.start();

	if( useShared )
	    vizEnv = env;
	else
	    vizEnv = boost::shared_ptr<envire::Environment>( new envire::Environment() );
	viz.getWidget()->setEnvironment( vizEnv.get() );
    }
#endif

#ifndef DEBUG_VIZ
    if( _debub_viz.value() )
	throw std::runtime_error( "DEBUG_VIZ not compiled in." );
#endif

    // init the filter
    base::Pose pose( _start_pose.value().position, _start_pose.value().orientation );
    std::cerr << "starting at position " << pose.position.transpose() << std::endl;
    filter->init( env.get(), pose, useShared );

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

