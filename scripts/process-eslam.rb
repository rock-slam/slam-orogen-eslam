#! /usr/bin/env ruby
require 'orocos/log'
require 'rock/bundle'
require 'vizkit'
include Orocos

class Eslam
    attr_accessor :env_dir, :log_dir, :eslam_config

    def initialize
	Bundles.initialize unless Orocos.initialized?
	@debug_viz = false
    end

    def init_viz
	#@debug_viz = true
	@view3d = Vizkit.vizkit3d_widget
	@contact_viz = Vizkit.default_loader.BodyContactStateVisualization
	@view3d.setPluginDataFrame( "body", @contact_viz )
	@pose_viz = Vizkit.default_loader.RigidBodyStateVisualization
	@odometry_viz = Vizkit.default_loader.RigidBodyStateVisualization
	@trajectory_viz = Vizkit.default_loader.TrajectoryVisualization
	@envire_viz = Vizkit.default_loader.EnvireVisualization
	@envire_viz.load @env_dir if @env_dir
	@view3d.setVisualizationFrame("world")
    end

    def from_logs( logs )
	@log = Orocos::Log::Replay.open( logs )

	# assume asguard simulation for now...
	# make more generic in the future for other logs
	simulation = true

	if simulation
	    #@laser_samples = @log.asguard_simulation.scans
	    @contact_samples = @log.asguard_simulation.contact_samples
	    @reference_pose = @log.asguard_simulation.true_pose
	    @odometry = @log.contact_odometry.odometry_samples

	    # get start pose from first reference pose sample
	    @start_pose = @reference_pose.stream.first[2]

	    # load the transformer configuration
	    Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_simulator_scripts.rb'))

	    # setup logs in nameservice for transformer setup to work
	    ns = Orocos::Local::NameService.new
	    ns.register( @log.contact_odometry, "odometry" )
	    ns.register( @log.asguard_simulation, "asguard_simulation" )
	    Orocos.name_service << ns
	else
	end
    end

    def configure eslam
	props = ['default']
	if @env_dir
	    props << 'localization'
	end

	# load config file if specified, otherwise use the bundles default configuration
	if @eslam_config
	    Orocos.apply_conf( eslam, @eslam_config, props, :override => true )
	else
	    Orocos.conf.apply( eslam, props, true )
	end
    end

    def run
	Bundles.run 'eslam::Task' => 'eslam', :valgrind => false, :output => nil do |p|
	    eslam = Bundles.get('eslam')

	    # connect task to data sources
	    @laser_samples.connect_to( eslam.scan_samples, :type => :buffer, :size => 1000 ) if @laser_samples
	    @contact_samples.connect_to( eslam.bodystate_samples, :type => :buffer, :size => 1000 )
	    #log.camera_left.frame.connect_to( eslam.terrain_classification_frames, :type => :buffer, :size => 1000 )
	    #log.stereo.stereo_features.connect_to( eslam.stereo_features, :type => :buffer, :size => 2 )

	    # configure the eslam module
	    configure eslam

	    Bundles.transformer.setup( eslam )

	    # handle start position:
	    # if there is a reference pose stream, take the
	    # first sample from there, so the two are comparable
#	    @reference_pose.connect_to( eslam.reference_pose, :type => :buffer, :size => 10 ) do |data,_|
#		data.position = data.position - start_pos
#		data
#	    end
	    eslam.start_pose = @start_pose
#	    do |p|
#		p.position = @start_pose.position
#		p.orientation = @start_pose.orientation
#	    end

	    # set other configuration options
	    eslam.debug_viz = @debug_viz 
	    eslam.environment_path = @env_dir if @env_dir

	    eslam.transformer_max_latency = 2.0

	    eslam.configure
	    eslam.start

	    if @view3d then
		Vizkit.display eslam.pose_distribution 
		Vizkit.display @reference_pose, :widget => "RigidBodyStateVisualization"
		@odometry.connect_to @odometry_viz do |rbs,_|
		    rbs.position = rbs.position + @start_pose.position 
		end
		@contact_samples.connect_to @contact_viz
		eslam.pose_samples.connect_to do |rbs,_|
		    @pose_viz.updateRigidBodyState( rbs )
		    @trajectory_viz.updateTrajectory( rbs.position )
		    rbs
		end
		@view3d.show

		Vizkit.control @log
		Vizkit.exec
	    else
		Orocos.log_all_ports( (@log_dir && {:log_dir => @log_dir}) || {} )
		log.run
	    end
	end
    end
end

