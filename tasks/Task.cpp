#include "Task.hpp"
#include <algorithm>

#include <envire/operators/MergeMLS.hpp>
#include <envire/tools/GraphViz.hpp>
#include <envire/Orocos.hpp>
#include <envire/tools/ExpectationMaximization.hpp>
#include <envire/tools/GaussianMixture.hpp>

using namespace eslam;

Task::Task(std::string const& name)
    : TaskBase(name), orocosEmitter(NULL), mapFilter(NULL)
{
    _start_pose.value().invalidate();
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), orocosEmitter(NULL), mapFilter(NULL)
{
    _start_pose.value().invalidate();
}

base::samples::RigidBodyState Task::cloneMap()
{
    // this function will first find the particle with
    // the heighest weight, then merge all the individual grids
    // into one compound grid, copied to a new environment
    // and then either dumped to file or written to the
    // output port
    assert( !filter->getParticles().empty() );

    // first get the map pointer to the best particle
    size_t best_idx = filter->getBestParticleIndex();
    envire::MLSMap::Ptr map = 
	filter->getParticles()[best_idx].grid.getMap();
    base::samples::RigidBodyState pose;
    pose.setTransform(
	filter->getParticles()[best_idx].getPose( update_orientation ) );

    // see if the map actually has any children (grids)
    std::list<envire::Layer*> grids = env->getChildren( map.get() );
    if( grids.empty() )
	throw std::runtime_error("map does not contain any grids.");
    // and get the resolution of the grid
    envire::MLSGrid* firstGrid = dynamic_cast<envire::MLSGrid*>( grids.front() );
    assert( firstGrid );
    double source_res = std::min( firstGrid->getScaleX(), firstGrid->getScaleY() );
    double res = _map_resolution.value();
    if( !(res > .0) )
	res = source_res;

    // get bounds of the map, and create a new grid based on this 
    Eigen::AlignedBox<double,2> extents = map->getExtents();
    size_t width = extents.sizes().x() / res;
    size_t height = extents.sizes().y() / res;
    envire::MLSGrid::Ptr target = 
	new envire::MLSGrid( std::max(1ul, width), std::max(1ul, height), 
		res, res, extents.min().x(), extents.min().y() );
    target->setUniqueId(_map_id.value());

    // attach map 
    envire::Environment *env = map->getEnvironment();
    env->setFrameNode( target.get(), env->getRootNode() );

    // setup and call merge operator
    envire::MergeMLS::Ptr mergeOp = new envire::MergeMLS();
    env->attachItem( mergeOp.get() );
    mergeOp->addInput( map.get() );
    mergeOp->addOutput( target.get() );
    mergeOp->setReverse( res < source_res );
    mergeOp->updateAll();

    env->detachItem( target.get() );
    env->detachItem( mergeOp.get() );

    // generate new Environment and move target 
    // mls to new environment
    envire::Environment target_env;
    target_env.setFrameNode( target.get(), target_env.getRootNode() );
    
    // add pose as framenode
    envire::FrameNode *bfn = new envire::FrameNode( pose );
    bfn->setUniqueId("/frame/body");
    target_env.addChild( target_env.getRootNode(), bfn );

    if( !_map_file.value().empty() )
	target_env.serialize( _map_file.value() );

    if (_map.connected())
    {
        envire::OrocosEmitter emitter(&target_env, _map);
	emitter.setTime( lastMapUpdate );
        emitter.flush();
    }

    return pose;
}

void Task::body2OdometryTransformerCallback(const base::Time& ts)
{
    Eigen::Affine3d body2odometry;
    if( !_body2odometry.get( ts, body2odometry ) )
	return;

    // set timestamp for envire eventdispatcher
    lastMapUpdate = ts;
    if( orocosEmitter )
	orocosEmitter->setTime( lastMapUpdate );

    // call the filter with the bodystate
    bool updated = filter->update( body2odometry, lastContactState, terrainClassificationWheel ); 

    // write result to output port
    base::Affine3d centroid = filter->getCentroid();

    // record bodystate and orientation when an update 
    // has occured. this is mainly for visualizing purposes,
    // so to capture the updated state.
    if( updated )
    {
	// also clear the terrain classification data collected so far
	terrainClassificationWheel.clear();

	update_orientation = body2odometry.linear();
	update_bodystate = lastContactState;
    }

    // update debug information
    // this may updated the centroid depending on configuration
    updateFilterInfo( ts, lastContactState, centroid, updated  );

    // write the centroid to the output port as the current best guess
    base::samples::RigidBodyState res_rbs;
    res_rbs.time = ts;
    res_rbs.sourceFrame = _body_frame.get();
    res_rbs.targetFrame = _world_frame.get();
    res_rbs.setTransform( centroid );
    res_rbs.sourceFrame = "body";
    res_rbs.targetFrame = "world";

    _pose_samples.write( res_rbs );
    outputCounter++;
}

void Task::bodystate_samplesTransformerCallback(const base::Time &ts, const ::odometry::BodyContactState &bodystate_samples_sample)
{
    lastContactState = bodystate_samples_sample;
}

void Task::terrain_classification_framesTransformerCallback(const base::Time &ts, const ::base::samples::frame::Frame &terrain_classification_frames_sample)
{
    // just store the sample and process in other callback
    terrainClassificationFrame = terrain_classification_frames_sample;
}

void Task::terrain_classification_wheelTransformerCallback(const base::Time &ts, const ::terrain_estimator::TerrainClassification &terrain_classification_wheel_sample)
{
    // the terrain classification data is stored in a vector, which gets
    // cleared when the filter has processed the data this means, that the data
    // might by slightly out of sync. Since the data is very rough anyway, this
    // should be ok.  The alternative would be to perform an asynchronous
    // filter update, which is more costly.
    terrainClassificationWheel.push_back( terrain_classification_wheel_sample );
}


void Task::distance_framesTransformerCallback(const base::Time &ts, const ::base::samples::DistanceImage &distance_frames_sample)
{
    if( doMapping )
    {
	Eigen::Affine3d body2odometry, lcamera2body;
	if( !_body2odometry.get( ts, body2odometry ) || !_lcamera2body.get( ts, lcamera2body ) )
	    return;

	// update the filter with laser data
	if( terrainClassificationFrame.time == distance_frames_sample.time )
	    filter->update( body2odometry, distance_frames_sample, lcamera2body, &terrainClassificationFrame );
	else
	    filter->update( body2odometry, distance_frames_sample, lcamera2body );
    }
}

void Task::scan_samplesTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_samples_sample)
{
    if( doMapping )
    {
	Eigen::Affine3d body2odometry, laser2body;
	if( !_body2odometry.get( ts, body2odometry ) || !_laser2body.get( ts, laser2body ) )
	    return;

	// update the filter with laser data
	filter->update( body2odometry, scan_samples_sample, laser2body );
    }
}

void Task::updateViewFilter()
{
    if( _debug_viz.value() )
    {
	const int inspect_particle_idx = viz.getWidget()->getInspectedParticleIndex();
	size_t view_idx = filter->getBestParticleIndex();
	if( inspect_particle_idx >= 0 )
	    view_idx = inspect_particle_idx;

	viz.getWidget()->viewMap( view_idx );
    }

    if( _envire_data.connected() && mapFilter )
    {
	std::vector<eslam::PoseEstimator::Particle>& particles( filter->getParticles() );
	size_t view_idx = filter->getBestParticleIndex();
	envire::MLSMap *map = 
	    particles[view_idx].grid.getMap();

	mapFilter->viewMap( map );
    }
}

void Task::updateFilterInfo( const base::Time& ts, const odometry::BodyContactState& bs, base::Affine3d& centroid, bool updated )
{
    if( _debug_viz.value() || _pose_distribution.connected() )
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

	if( _calc_gmm.get() )
	{
	    std::vector<base::Vector2d> em_pars;
	    std::vector<double> em_weights;
	    envire::ExpectationMaximization<PoseDistribution::GMM> em;
	    for( size_t i=0; i<pd.particles.size(); i++ )
	    {
		em_pars.push_back( particles[i].position );
		em_weights.push_back( particles[i].weight );
	    }
	    em.initialize( 5, em_pars, em_weights );
	    em.run( 1e-5, 10 );
	    pd.gmm.params.swap( em.gmm.params );


	    /*
	    // take the largest of the gaussians and use it as the new centroid
	    int max_idx = -1;
	    int max_weight = 0;
	    for( size_t i=0; i < pd.gmm.params.size(); i++ )
	    {
		if( pd.gmm.params[i].weight > max_weight )
		{
		    max_idx = i;
		    max_weight = pd.gmm.params[i].weight;
		}
	    }
	    if( max_idx >=0 )
	    {
		centroid.matrix().topRightCorner<2,1>() = pd.gmm.params[max_idx].dist.mean;
	    }
	    */
	}

	if( _pose_distribution.connected() )
	{
	    const unsigned int period = _eslam_config.value().logParticlePeriod;
	    if( period > 0 && (outputCounter % period) == 0 )
		_pose_distribution.write( pd );
	}

	if( _debug_viz.value() )
	{
	    viz.getWidget()->setPoseDistribution( pd );
	    viz.getWidget()->setBodyState( bs );
	    viz.getWidget()->setCentroidPose( base::Pose( centroid ) );
	    base::samples::RigidBodyState ref_pose;
	    while( _reference_pose.read( ref_pose ) == RTT::NewData )
		viz.getWidget()->setReferencePose( ref_pose.getPose() );
	}
    }

    if( updated )
	updateViewFilter();
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    // initialize the filter object with the current configuration
    filter = boost::shared_ptr<eslam::EmbodiedSlamFilter>( new eslam::EmbodiedSlamFilter( 
		_odometry_config.get(), 
		_eslam_config.get() ) ); 

    // load an environment if path is specified
    useShared = false;
    if( !_environment_path.value().empty() )
    {
	std::cout << "loading environment: " << _environment_path.value() << std::endl;
	env = boost::shared_ptr<envire::Environment>( envire::Environment::unserialize( _environment_path.value() ) );
	doMapping = _eslam_config.get().useVisualUpdate;
	useShared = true;
    }
    else
    {
	env = boost::shared_ptr<envire::Environment>( new envire::Environment() );
	doMapping = true;
    }

    //register callback on new odometry readings
    _body2odometry.registerUpdateCallback(boost::bind(&Task::body2OdometryTransformerCallback, this, _1));

    return TaskBase::configureHook();
}

bool Task::startHook()
{
    if( !TaskBase::startHook() )
	return false;

    if( _debug_viz.value() )
    {
	viz.start();
	viz.getWidget()->setEnvironment( env.get() );
	viz.getWidget()->setPoseParticles( &filter->getParticles() );

	Eigen::Vector3d start_pose = _start_pose.value().position;
	viz.getWidget()->setCameraEye( start_pose.x(), start_pose.y(), start_pose.z() );
    }
    
    // init the filter
    base::Pose pose( _start_pose.value().position, _start_pose.value().orientation );
    std::cerr << "starting at position " << pose.position.transpose() << std::endl;
    filter->init( env.get(), pose, useShared, _hash_config.value() );

    std::cerr << "initialized" << std::endl;

    // reset output counter
    outputCounter = 0;
    
    return true;
}

void Task::updateHook()
{
    if( !orocosEmitter && _envire_data.connected() )
    {
	// register the binary event dispatcher, 
	// which will write envire data to a port
	orocosEmitter = new envire::OrocosEmitter( _envire_data );
	mapFilter = new vizkit::MapVizEventFilter();
	orocosEmitter->setFilter( mapFilter );
	orocosEmitter->useContextUpdates( env.get() );
	orocosEmitter->useEventQueue( true );
	orocosEmitter->attach( env.get() );
    }
    TaskBase::updateHook();

    if( orocosEmitter )
    {
	if( _envire_data.connected() )
	{
	    if( (lastEnvireDataUpdate + base::Time::fromSeconds(_envire_period.value())) < base::Time::now() ) 
	    {
		orocosEmitter->flush();
		lastEnvireDataUpdate = base::Time::now();
	    }
	}
	else
	{
	    delete orocosEmitter;
	    delete mapFilter;
	    orocosEmitter = NULL;
	    mapFilter = NULL;
	}
    }
}

// void Task::errorHook()
// {
// }

void Task::stopHook()
{
    delete orocosEmitter;
    delete mapFilter;
    orocosEmitter = NULL;
    mapFilter = NULL;

    // write environment, if path is given
    if( !_environment_debug_path.value().empty() )
    {
        env->serialize(_environment_debug_path.value() );
    }
}

// void Task::cleanupHook()
// {
// }

