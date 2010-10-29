#! /usr/bin/env ruby

require 'log_replay'
require 'widget_grid'

require 'orocos'
include Orocos

if ARGV.size < 2 then 
    puts "usage: process_logs.rb log_dir environment_path"
    exit
end

BASE_DIR = File.expand_path('..', File.dirname(__FILE__))
ENV['PKG_CONFIG_PATH'] = "#{BASE_DIR}/build:#{ENV['PKG_CONFIG_PATH']}"
Orocos.initialize

# This will kill processes when we quit the block
Orocos::Process.spawn('eslam_test') do |p|
    eslam = p.task('eslam')

    Orocos.log_all_ports #( {:log_dir => ARGV[0]} )

    log_replay = Pocosim::LogReplay.new( ['test.0.log', 'gps.0.log', 'xsens_imu.0.log', 'lowlevel.1.log'].map!{|x| File.join(ARGV[0], x)} )
    log_replay.odometry.odometry_samples.connect_to( eslam.orientation_samples, :type => :buffer, :size => 100 )
    log_replay.odometry.bodystate_samples.connect_to( eslam.bodystate_samples, :type => :buffer, :size => 100 )

    #start_pos = log_replay.gps.position_samples.stream.first[2]
    #start_pos.orientation = log_replay.xsens_imu.orientation_samples.stream.first[2].orientation

    start_pos = log_replay.pose_estimator.pose_samples.stream[10][2]

    eslam.environment_path = ARGV[1]
    eslam.start_pose = start_pos
    eslam.configure
    eslam.start

    widget_grid = WidgetGrid.new
    widget_grid.control( log_replay )
    widget_grid.run

    #log_replay.align()
    #log_replay.run(false)
end

