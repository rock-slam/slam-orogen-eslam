#! /usr/bin/env ruby

#require 'widget_grid'

require 'orocos'
require 'orocos/log'
include Orocos

# will generate the output of the odometry module
# for a given log directory

if ARGV.size < 1 then 
    puts "usage: process_logs.rb log_dir"
    exit
end

BASE_DIR = File.expand_path('..', File.dirname(__FILE__))
ENV['PKG_CONFIG_PATH'] = "#{BASE_DIR}/build:#{ENV['PKG_CONFIG_PATH']}"
Orocos.initialize

# This will kill processes when we quit the block
Orocos.run 'lowlevel' do |p|
    odometry = p.task('odometry')
    odometry.configure
    odometry.start

    log_replay = Log::Replay.open( ARGV[0] )
    log_replay.hbridge.status.connect_to( odometry.hbridge_samples, :type => :buffer, :size => 1000 )
    log_replay.xsens_imu.orientation_samples.connect_to( odometry.orientation_samples, :type => :buffer, :size => 100 )
    log_replay.sysmon.system_status.connect_to( odometry.systemstate_samples, :type => :buffer, :size => 100 )

    Orocos.log_all_ports( {:tasks => 'odometry', :log_dir => ARGV[0]} )

    log_replay.align()

    while log_replay.step(false) do end
end

