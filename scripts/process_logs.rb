#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'rock/bundle'
require 'vizkit'
include Orocos

opts = {}
p = OptionParser.new do |o|
    o.banner = "usage: process_gnuplot.rb [options] log_dir"
    o.on("-e", "--env=PATH", "Path to environment") {|v| opts[:env_dir] = v }
    o.on("-o", "--out=PATH", "Output directory (will be created)") {|v| opts[:out_dir] = v }
    o.on("-d", "--debug_viz", "Use visualization for debugging") {|v| opts[:debug] = v }
    o.on("-c", "--configuration=NAME", "Name of the configuration to use") {|v| opts[:configuration] = v }
    o.on("-h", "--help", "Show this message") do
	puts o
	exit
    end
end
p.parse!(ARGV)
if ARGV.empty?
    puts p 
    exit
end
opts[:log_dir] = ARGV[0]

Bundles.initialize
Nameservice::enable(:Local)

# This will kill processes when we quit the block
Bundles.run 'eslam::Task' => 'eslam', :valgrind => false, :output => nil do |p|
    eslam = p.task('eslam')

    log = Orocos::Log::Replay.open( opts[:log_dir] )
    Nameservice::Local.registered_tasks["odometry"] = log.contact_odometry
    Nameservice::Local.registered_tasks["dynamixel"] = log.dynamixel

    Orocos.conf.apply( eslam )
    Bundles.transformer.setup( eslam )

    log.hokuyo.scans.connect_to( eslam.scan_samples, :type => :buffer, :size => 1000 ) 
    log.contact_odometry.odometry_samples.connect_to( eslam.orientation_samples, :type => :buffer, :size => 10000 )
    log.asguard_body.contact_samples.connect_to( eslam.bodystate_samples, :type => :buffer, :size => 1000 )
    #log.camera_right.frame.connect_to( eslam.terrain_classification_frames, :type => :buffer, :size => 1000 )

    #log.mb500.position_samples.connect_to( @eslam.reference_pose, :type => :buffer, :size => 10 )
    log.contact_odometry.odometry_samples.connect_to( eslam.reference_pose, :type => :buffer, :size => 1000 )
    #log.dynamixel.lowerDynamixel2UpperDynamixel.connect_to( @eslam.dynamic_transformations, :type => :buffer, :size => 1000 )
    #log.stereo.distance_frame.connect_to( eslam.distance_frames, :type => :buffer, :size => 2 )

    eslam.debug_viz = true if opts[:debug]

    eslam.transformer_max_latency = 2.0

    eslam.configure
    eslam.start

    if opts[:debug] then
	#view3d = Vizkit.vizkit3d_widget
	#contact_viz = Vizkit.default_loader.BodyContactStateVisualization.plugins['BodyContactStateVisualization']
	#Vizkit.display eslam.contact_samples, :widget => view3d
	#view3d.show

	Vizkit.control log
	Vizkit.exec
    else
	Orocos.log_all_ports( {:log_dir => opts[:log_dir]} )
	log.run
    end
end

