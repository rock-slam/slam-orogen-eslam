require 'vizkit'
require 'orocos'
include Orocos

Orocos.initialize

class SequenceArray
    class Sequence
	def initialize( samples = [], times = [] )
	    @idx = 0
	    @samples = samples 
	    @times = times 
	end

	def clone 
	    Sequence.new @idx.clone, @samples.clone
	end

	def push sample, ts
	    @samples << sample
	    @times << ts
	end

	def current_time
	    @times[@idx] if @idx < @samples.size
	end

	def current_time=( val )
	    @times[@idx] = val
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

	attr_accessor :idx, :samples, :times
    end

    def initialize( seq = {} )
	@seq = seq 
    end

    def clone
	SequenceArray.new @seq.clone
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
    def initialize( opts )
	@opts = opts
	@log_dir = opts[:log_dir]
	@environment_path = opts[:env_dir]
	@seed = 42
    end

    def start
	@log_replay.reset_time_sync
	while @log_replay.step(true) do 
	    # need to call process events here 
	    # for the reader callbacks to work
	    Vizkit.process_events
	end
    end

    def info( os = $stdout )
	prev = $>
	$> = os
	puts "log_dir: #{@log_dir}"
	puts "env_path: #{@environment_path}"
	puts "start_pos: #{@start_pos.position}"
	pp @errors
	@configs.each do |c|
	    pp c
	end
	$> = prev
    end

    def params
	# store all configuration objects in this array
	# for logging
	@configs = {} 

	# eslam_config
	config = @eslam.eslam_config.dup
	config.seed = @seed
	config.measurementThreshold.distance = 0.05
	config.measurementThreshold.angle = 5.0 * Math::PI/180.0
	config.initialError = 0.5
	config.particleCount = 250
	config.minEffective = 50
	config.measurementError = 0.15
	config.discountFactor = 0.95
	config.spreadThreshold = 0.5
	config.spreadTranslationFactor = 0.0001
	config.spreadRotationFactor = 0.00005
	config.slipFactor = 0.2
	@configs[:eslam] = config

	# odometry config
	config = @eslam.odometry_config.dup
	@eslam.odometry_config = config
	config.seed = @seed
	config.constError.translation = Eigen::Vector3.new( 0.01, 0.01, 0.0 )
	config.constError.yaw = 5e-3 

	config.distError.translation = Eigen::Vector3.new( 0.1, 0.5, 0.0 )
	config.distError.yaw = 1e-3 

	config.tiltError.translation = Eigen::Vector3.new( 0.0, 0.0, 0.0 )
	config.tiltError.yaw = 0

	config.dthetaError.translation = Eigen::Vector3.new( 0.05, 0.01, 0.0 )
	config.dthetaError.yaw = 0.005
	@configs[:odometry] = config

	# asguard config
	config = @eslam.asguard_config.dup
	@configs[:asguard] = config
    end

    def configure
	# initialize the @configs array with the parameters for 
	# each of the configurations
	params

	# write configurations to module
	@eslam.eslam_config = @configs[:eslam]
	@eslam.odometry_config = @configs[:odometry] 
	@eslam.asguard_config = @configs[:asguard] 

	# get start position from gps
	@start_pos = @log_replay.mb500.position_samples.stream.first[2]
	@start_pos.orientation = @log_replay.xsens_imu.orientation_samples.stream.first[2].orientation
	@start_pos.position -= antenna_correction( @start_pos.orientation )
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
	#Orocos::Process.spawn('eslam_test', :valgrind => true, :valgrind_options => ["--track-origins=yes"] ) do |p|
	Orocos::Process.spawn('eslam_test') do |p|
	    @eslam = p.task('eslam')
	    #Orocos.log_all_ports #( {:log_dir => ARGV[0]} )

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
		    0.01
		else
		    0
		end
	    end

	    @eslam.configure
	    @eslam.start

	    start
	end

	correct_result
	calc_error
	#pp @errors
	puts "\nmean filter\tmean odo\tmax filter\tmax odo"
	puts "%.2f\t%.2f\t%.2f\t%.2f" % [ 
	    @errors[:centroid][:mean_error], 
	    @errors[:odometry][:mean_error], 
	    @errors[:centroid][:max_error], 
	    @errors[:odometry][:max_error] ]

	puts "dist filter\tdist odo\tdist gps"
	puts "%.2f\t%.2f\t%.2f" % [ 
	    @errors[:centroid][:dist], 
	    @errors[:odometry][:dist], 
	    @errors[:gps][:dist] ]
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

    def gps_time_offset
	# debug code to find the offset in the gps time
	times = @result.seq[:gps].times.clone

	File.open("/tmp/gps_time_offset.txt", "w") do |f|
	    (-1.0..1.0).step(0.02) do |offset|
		r = times.map {|t| t + offset} 
		@result.seq[:gps].times = r

		calc_error
		f.puts "#{offset} #{@errors[:centroid][:mean_error]}"
	    end
	end

	@result.seq[:gps].times = times
    end

    def calc_error
	plots = [:centroid, :odometry, :gps]
	@errors = {}
	plots.each {|s| @errors[s] = {:count => 0, :sum_error => 0, :dist => 0, :max_error => 0} }

	@result.each(:gps) do |r|
	    ref_pos = r.seq[:gps].current
	    @errors.each do |s,v| 
		pos = r.seq[s].current
		v[:count] += 1
		error = (pos - ref_pos).norm
		v[:max_error] = error if error > v[:max_error]
		v[:final_error] = error
		v[:sum_error] += error 
		if v[:last_pos] 
		    v[:dist] += (pos - v[:last_pos]).norm
		end
		v[:last_pos] = pos
	    end
	end
	@errors.each {|k,v| v[:mean_error] = v[:sum_error] / v[:count]}
    end
end
