#include "Task.hpp"
#include <algorithm>

#include <envire/tools/GraphViz.hpp>
#include <eslam/ExpectationMaximization.hpp>

using namespace eslam;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

void Task::bodystate_samplesTransformerCallback(const base::Time &ts, const ::asguard::BodyState &bodystate_samples_sample)
{
    Eigen::Affine3d body2odometry;
    if( !_body2odometry.get( ts, body2odometry ) )
	return;

    // call the filter with the bodystate
    bool updated = filter->update( body2odometry, bodystate_samples_sample ); 

    // write result to output port
    base::Affine3d centroid = filter->getCentroid();

    base::samples::RigidBodyState res_rbs;
    res_rbs.time = ts;
    res_rbs.setTransform( centroid );

    _pose_samples.write( res_rbs );

    // record bodystate and orientation when an update 
    // has occured. this is mainly for visualizing purposes,
    // so to capture the updated state.
    if( updated )
    {
	update_orientation = body2odometry.linear();
	update_bodystate = bodystate_samples_sample;
    }

    // update debug information
    updateFilterInfo( ts, bodystate_samples_sample, centroid, updated  );
}

void Task::distance_framesTransformerCallback(const base::Time &ts, const ::dense_stereo::distance_image &distance_frames_sample)
{
    //throw std::runtime_error("Transformer callback for distance_frames not implemented");
    Eigen::Affine3d body2odometry, lcamera2body;
    if( !_body2odometry.get( ts, body2odometry ) || !_lcamera2body.get( ts, lcamera2body ) )
	return;

    // update the filter with laser data
    filter->update( body2odometry, distance_frames_sample, lcamera2body );
}

void Task::scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample)
{
    if( useScans )
    {
	Eigen::Affine3d body2odometry, laser2body;
	if( !_body2odometry.get( ts, body2odometry ) || !_laser2body.get( ts, laser2body ) )
	    return;

	// update the filter with laser data
	filter->update( body2odometry, scan_samples_sample, laser2body );
    }
}

void Task::updateFilterInfo( const base::Time& ts, const asguard::BodyState& bs, const base::Affine3d& centroid, bool updated )
{
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

	std::vector<base::Vector2d> em_pars;
	std::vector<double> em_weights;
	eslam::ExpectationMaximization<PoseDistribution::GMM> em;
	for( size_t i=0; i<pd.particles.size(); i++ )
	{
	    em_pars.push_back( particles[i].position );
	    em_weights.push_back( particles[i].weight );
	}
	em.initialize( 5, em_pars, em_weights );
	em.run( 1e-5, 10 );
	pd.gmm.params.swap( em.gmm.params );

	if( _pose_distribution.connected() )
	    _pose_distribution.write( pd );

#ifdef DEBUG_VIZ
	if( _debug_viz.value() )
	{
	    viz.getWidget()->setPoseDistribution( pd );
	    viz.getWidget()->setBodyState( bs );
	    viz.getWidget()->setCentroidPose( base::Pose( centroid ) );
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
    if( _debug_viz.value() )
	throw std::runtime_error( "DEBUG_VIZ not compiled in." );
#endif

    // init the filter
    base::Pose pose( _start_pose.value().position, _start_pose.value().orientation );
    std::cerr << "starting at position " << pose.position.transpose() << std::endl;
    filter->init( env.get(), pose, useShared );

    std::cerr << "initialized" << std::endl;

    return TaskBase::configureHook();
}

bool Task::startHook()
{
    return true;
}

void Task::updateHook()
{
    // this is to be backward compatible
    base::samples::RigidBodyState body2odometry;
    while( _orientation_samples.read( body2odometry ) == RTT::NewData )
	transformer.pushDynamicTransformation( body2odometry );

    TaskBase::updateHook();

    // only write output if the aggregator actually had some data
    // this is slightly implicit, and could be made more explicit in
    // the aggregator
    if( transformer.getStatus().latest_time > base::Time() )
	_streamaligner_status.write( transformer.getStatus() );
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

