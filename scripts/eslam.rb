require 'vizkit'
require 'orocos'
require 'sequence_array'
require 'asguard'
require 'eslam_config'
include Orocos

Orocos.initialize
Orocos::Log.logger.level = Logger::WARN

module Eslam
class Replay
    def initialize( opts )
	@opts = opts
	@log_dir = opts[:log_dir]
	@environment_path = opts[:env_dir]
	@config_name = :mapping
	@config_name = opts[:configuration].to_sym if opts[:configuration] 
	@seed = 42

	puts "INFO: using configuration #{@config_name}"
    end

    def start
	@replay.log.reset_time_sync
	while @replay.log.step(true) do 
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
	pp @config
	$> = prev
    end

    def configure
	# configure the object
	@config = Config.new @seed
	@config.update @eslam, @config_name 

	@start_pos = @eslam.start_pose 
	if @replay.use? :gps
	    # get start position from gps
	    @start_pos = @replay.log.mb500.position_samples.stream.first[2]
	    @start_pos.orientation = @replay.log.xsens_imu.orientation_samples.stream.first[2].orientation
	    @start_pos.position -= antenna_correction( @start_pos.orientation )
	    @eslam.start_pose = @start_pos
	end

	@eslam.calc_gmm = true

	# set environment path if available
	@eslam.environment_path = @environment_path if @environment_path 

	# setup the hooks, so that we can store the relevant output values
	@result = SequenceArray.new

	centroid = @result.add(:centroid)
	@eslam.pose_samples.connect_to :type => :buffer,:size => 100 do |data, name| 
	    centroid.push( data.position, data.time ) if data
	    data
	end

	if @replay.use? :gps
	    gps = @result.add(:gps)
	    @replay.log.mb500.position_samples.connect_to :type => :buffer,:size => 100 do |data, name|
		gps.push( data.position, data.time ) if data
		data
	    end
	end

	odometry = @result.add(:odometry)
	orientation = @result.add(:orientation)
	@replay.log.odometry.odometry_samples.connect_to :type => :buffer,:size => 100 do |data, name|
	    if data
		@odometry_offset ||= @start_pos.position - data.position 
		odometry_pos = data.position + @odometry_offset
		odometry.push( odometry_pos, data.time )
		orientation.push( data.orientation, data.time )
	    end
	    data
	end
    end

    def setup_transforms

    end

    def run
	# This will kill processes when we quit the block
	#Orocos::Process.spawn('eslam_test', :valgrind => true, :valgrind_options => ["--track-origins=yes"] ) do |p|
	Orocos::Process.spawn('eslam_test') do |p|
	    @eslam = p.task('eslam')
	    #Orocos.log_all_ports #( {:log_dir => ARGV[0]} )

	    @replay = Asguard::Replay.new( @log_dir )
	    #@replay.disable :gps
	    tf = Asguard::Transform.new
	    tf.use :dynamixel if @replay.has? :dynamixel 
	    tf.setup_filters @replay

	    @replay.log.hokuyo.scans.connect_to( @eslam.scan_samples, :type => :buffer, :size => 1000 ) 
	    @replay.log.odometry.odometry_samples.connect_to( @eslam.orientation_samples, :type => :buffer, :size => 1000 )
	    @replay.log.odometry.bodystate_samples.connect_to( @eslam.bodystate_samples, :type => :buffer, :size => 1000 )
	    
	    if @replay.use? :gps 
		@replay.log.mb500.position_samples.connect_to( @eslam.reference_pose, :type => :buffer, :size => 10 )
		puts "INFO: Using GPS Position data for reference."
	    else
		@replay.log.odometry.odometry_samples.connect_to( @eslam.reference_pose, :type => :buffer, :size => 1000 )
	    end
	    if @replay.use? :dynamixel
		@replay.log.dynamixel.lowerDynamixel2UpperDynamixel.connect_to( @eslam.dynamic_transformations, :type => :buffer, :size => 1000 )
		puts "INFO: Using dynamic transformation chain for sensor head."
	    end
	    if @replay.use? :stereo 
		@replay.log.stereo.distance_frame.connect_to( @eslam.distance_frames, :type => :buffer, :size => 2 )
		puts "INFO: Using distance images."
	    end

	    configure
	    threshold = 0.5
	    status_reader = @eslam.transformer_status.reader

	    latest_time = nil
	    @replay.log.align( :use_sample_time )
#	    @replay.log.time_sync do |current_time, actual_time_delta, required_time_delta|
#		module_time = status_reader.read_new
#		if !module_time.nil?
#		    latest_time = module_time.latest_time
#		end
#
#		@re_start_time ||= current_time
#		@re_stop_time = current_time
#
#		if latest_time and latest_time + threshold < current_time
#		    0.01
#		else
#		    0
#		end
#	    end

	    @eslam.configure
	    @eslam.start
	    tf.configure_task @eslam

	    @pr_start_time = Time.now

	    start
	end

	@pr_stop_time = Time.now

	puts "\nruntime: #{@pr_stop_time-@pr_start_time}"
	puts "logtime: #{@re_stop_time-@re_start_time}"

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
end
