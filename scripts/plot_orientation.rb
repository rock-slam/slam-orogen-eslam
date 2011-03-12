#! /usr/bin/env ruby

require 'vizkit'

if ARGV.size < 1 then 
    puts "usage: plot_orientation.rb log_dir"
    exit
end

log_replay = Orocos::Log::Replay.open( ARGV[0] )
log_replay.odometry.odometry_samples :type => :buffer, :size => 100 do |sample|
    puts (sample.orientation.to_euler(2,1,0)*(180.0/Math::PI)).to_a.join(' ')
    sample
end

log_replay.align()
while log_replay.step(false) do Vizkit.process_events end

