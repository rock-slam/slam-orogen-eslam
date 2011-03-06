require 'vizkit'
require 'orocos'
include Orocos

Orocos.initialize

class SequenceArray
    class Sequence
	def initialize
	    @idx = 0
	    @samples = []
	    @times = []
	end

	def push sample, ts
	    @samples << sample
	    @times << ts
	end

	def current_time
	    @times[@idx] if @idx < @samples.size
	end

	def next_time
	    @times[@idx+1] if (@idx+1) < @samples.size
	end

	def current
	    @samples[@idx]
	end

	def current=( val )
	    @samples[@idx] = val
	end

	def advance
	    @idx += 1 unless @idx >= @samples.size
	    current_time
	end

	attr_accessor :idx, :samples
    end

    def initialize
	@seq = {} 
    end

    def add( name )
	@seq[name] = Sequence.new
    end

    def advance( name )
	if t = @seq[name].advance
	    @seq.each do |sn, sv|
		while sv.next_time and sv.next_time < t do sv.advance end
	    end
	end
    end

    def each( name )
	reset
	return unless @seq[name].current
	    
	begin
	    yield self
	end while advance(name)
    end

    def reset
	@seq.each do |sn, sv|
	    sv.idx = 0
	end
    end

    attr_accessor :seq
end


class Eslam
    def initialize( log_dir, environment_path = nil )
	@log_dir = log_dir
	@environment_path = environment_path
    end

    def start
	@log_replay.run
    end

    def info( os = $stdout )
	prev = $>
	$> = os
	puts "log_dir: #{@log_dir}"
	puts "env_path: #{@environment_path}"
	puts "start_pos: #{@start_pos.position}"
	@configs.each do |c|
	    pp c
	end
	$> = prev
    end

    def configure
	# store all configuration objects in this array
	# for logging
	@configs = {} 

	# eslam_config
	config = @eslam.eslam_config.dup
	config.measurementThreshold.distance = 0.05
	config.measurementThreshold.angle = 5.0 * Math::PI/180.0
	config.mappingThreshold.distance = 0.02
	config.mappingThreshold.angle = 5.0 * Math::PI/180.0
	config.initialError = 0.5
	config.particleCount = 500
	config.minEffective = 100
	@eslam.eslam_config = config
	@configs[:eslam] = config

	# odometry config
	config = @eslam.odometry_config.dup
	@eslam.odometry_config = config
	config.yawError = 1e-3
	config.errorFactor = 2.0
	@configs[:odometry] = config

	# asguard config
	config = @eslam.asguard_config.dup
	@eslam.asguard_config = config
	@configs[:asguard] = config

	# get start position from gps
	@start_pos = @log_replay.mb500.position_samples.stream.first[2]
	@start_pos.orientation = @log_replay.xsens_imu.orientation_samples.stream.first[2].orientation
	@start_pos.position += antenna_correction( @start_pos.orientation )
	@eslam.start_pose = @start_pos

	# set environment path if available
	@eslam.environment_path = @environment_path if @environment_path 

	# setup the hooks, so that we can store the relevant output values
	@result = SequenceArray.new

	centroid = @result.add(:centroid)
	@eslam.pose_samples.connect_to :type => :buffer,:size => 100 do |data, name| 
	    centroid.push( data.position, data.time ) if data
	    data
	end

	gps = @result.add(:gps)
	@log_replay.mb500.position_samples.connect_to :type => :buffer,:size => 100 do |data, name|
	    gps.push( data.position, data.time ) if data
	    data
	end

	odometry = @result.add(:odometry)
	orientation = @result.add(:orientation)
	@log_replay.odometry.odometry_samples.connect_to :type => :buffer,:size => 100 do |data, name|
	    if data
		@odometry_offset ||= @start_pos.position - data.position 
		odometry_pos = data.position + @odometry_offset
		odometry.push( odometry_pos, data.time )
		orientation.push( data.orientation, data.time )
	    end
	    data
	end
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

	correct_result
    end

    def antenna_correction orientation
	# antenna correction is orientation specific
	orientation * Eigen::Vector3.new( 0, -0.425, 0.17 )
    end

    def correct_result
	@result.each(:orientation) do |r|
	    corr = antenna_correction r.seq[:orientation].current
	    [:centroid, :odometry].each do |s|
		r.seq[s].current += corr if r.seq[s].current
	    end
	end 
    end
end
