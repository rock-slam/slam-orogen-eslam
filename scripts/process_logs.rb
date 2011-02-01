#! /usr/bin/env ruby
require 'vizkit'

include Orocos

if ARGV.size < 1 then 
    puts "usage: process_logs.rb log_dir [environment_path]"
    exit
end

BASE_DIR = File.expand_path('..', File.dirname(__FILE__))
ENV['PKG_CONFIG_PATH'] = "#{BASE_DIR}/build:#{ENV['PKG_CONFIG_PATH']}"
Orocos.initialize

# This will kill processes when we quit the block
Orocos::Process.spawn('eslam_test') do |p|
    eslam = p.task('eslam')

    Orocos.log_all_ports #( {:log_dir => ARGV[0]} )

    log_replay = Orocos::Log::Replay.open( ['hokuyo.0.log','lowlevel.1.log'].map!{|x| File.join(ARGV[0], x)} )
    log_replay.hokuyo.scans.connect_to( eslam.scan_samples, :type => :buffer, :size => 100 ) 
    log_replay.odometry.odometry_samples.connect_to( eslam.orientation_samples, :type => :buffer, :size => 100 )
    log_replay.odometry.bodystate_samples.connect_to( eslam.bodystate_samples, :type => :buffer, :size => 100 )

    eslam.eslam_config.measurementThreshold.distance = 0.02
    eslam.eslam_config.measurementThreshold.angle = 5.0 * Math::PI/180.0
    eslam.eslam_config.mappingThreshold.distance = 0.02
    eslam.eslam_config.mappingThreshold.angle = 5.0 * Math::PI/180.0

    eslam.configure
    eslam.start

    threshold = 0.2
    status_reader = eslam.streamaligner_status.reader

    latest_time = nil
    log_replay.align( :use_sample_time )
    log_replay.time_sync do |current_time, actual_time_delta, required_time_delta|
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

    Vizkit.control log_replay
    Vizkit.exec
end

