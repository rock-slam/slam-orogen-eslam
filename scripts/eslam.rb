require 'vizkit'
require 'orocos'
include Orocos

Orocos.initialize

class Eslam
    def initialize( log_dir, environment_path = nil )
	@log_dir = log_dir
	@environment_path = environment_path
    end

    def start
	@log_replay.run
    end

    def configure
	config = @eslam.eslam_config
	config.measurementThreshold.distance = 0.05
	config.measurementThreshold.angle = 5.0 * Math::PI/180.0
	config.mappingThreshold.distance = 0.02
	config.mappingThreshold.angle = 5.0 * Math::PI/180.0
	config.initialError = 0.5
	config.particleCount = 250
	config.minEffective = 150
	@eslam.eslam_config = config

	@start_pos = @log_replay.mb500.position_samples.stream.first[2]
	@start_pos.orientation = @log_replay.xsens_imu.orientation_samples.stream.first[2].orientation
	@eslam.start_pose = @start_pos

	@eslam.environment_path = @environment_path if @environment_path 
    end

    def run
	# This will kill processes when we quit the block
	Orocos::Process.spawn('eslam_test') do |p|
	    @eslam = p.task('eslam')

	    Orocos.log_all_ports #( {:log_dir => ARGV[0]} )

	    @log_replay = Orocos::Log::Replay.open( @log_dir )
	    @log_replay.hokuyo.scans.connect_to( @eslam.scan_samples, :type => :buffer, :size => 1000 ) 
	    @log_replay.odometry.odometry_samples.connect_to( @eslam.orientation_samples, :type => :buffer, :size => 1000 )
	    @log_replay.odometry.bodystate_samples.connect_to( @eslam.bodystate_samples, :type => :buffer, :size => 1000 )
	    @log_replay.mb500.position_samples.connect_to( @eslam.reference_pose, :type => :buffer, :size => 10 )

	    configure

	    threshold = 0.5
	    status_reader = @eslam.streamaligner_status.reader

	    latest_time = nil
	    @log_replay.align( :use_sample_time )
	    @log_replay.time_sync do |current_time, actual_time_delta, required_time_delta|
		module_time = status_reader.read_new
		if !module_time.nil?
		    latest_time = module_time.latest_time
		end

		if latest_time and latest_time + threshold < current_time
		    0.1
		else
		    0
		end
	    end

	    @eslam.configure
	    @eslam.start

	    start
	end
    end
end
