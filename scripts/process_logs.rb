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
    eslam.environment_path = ARGV[1]
    eslam.configure
    eslam.start

    Orocos.log_all_ports #( {:log_dir => ARGV[0]} )

    log_replay = Pocosim::LogReplay.new( ['lowlevel.1.log'].map!{|x| File.join(ARGV[0], x)} )
    log_replay.odometry.odometry_samples.connect_to( eslam.orientation_samples, :type => :buffer, :size => 100 )
    log_replay.odometry.bodystate_samples.connect_to( eslam.bodystate_samples, :type => :buffer, :size => 100 )

    widget_grid = WidgetGrid.new
    widget_grid.control( log_replay )
    widget_grid.run

    #log_replay.align()
    #log_replay.run(false)
end

