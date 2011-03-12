#! /usr/bin/env ruby

require 'vizkit'

if ARGV.size < 2 then 
    puts "usage: plot_odometry_error.rb log_dir out_dir"
    exit
end
log_dir = ARGV[0]
out_dir = ARGV[1]

log_replay = Orocos::Log::Replay.open( log_dir )
log_replay.odometry.odometry_samples.tracked = true
log_replay.pose_estimator.pose_samples.tracked = true
log_replay.odometry.bodystate_samples.tracked = true

=begin
log_replay.odometry.odometry_samples :type => :buffer, :size => 100 do |sample|
    puts (sample.orientation.to_euler(2,1,0)*(180.0/Math::PI)).to_a.join(' ')
    sample
end
=end

odometry = log_replay.odometry.odometry_samples
bodystate = log_replay.odometry.bodystate_samples

last_gps = nil
last_odo = nil
last_orientation = nil
last_euler = nil
last_gps_euler = nil
wheels_old = nil

sum_error = 0
counter = 0

os = File.open( File.join(out_dir, "odo_error.dat"), "w" )
class Array
    def sum; inject( nil ) { |sum,x| sum ? sum+x : x }; end
    def mean; sum / size; end; 
end

log_replay.align(:use_sample_time)
log_replay.run do |port, data|
    if port.name == 'pose_samples' and odometry.read
	orientation = odometry.read.orientation
	wheels = bodystate.read.wheelPos.to_a

	euler = orientation.to_euler(2,1,0)
	odo_pos = odometry.read.position
	gps_pos = data.position
	gps_euler = data.orientation.to_euler(2,1,0)

	if last_gps and counter == 100
	    diff_gps = last_orientation.inverse * (gps_pos - last_gps)
	    diff_odo = last_orientation.inverse * (odo_pos - last_odo)
	    diff_theta_odo = euler[0] - last_euler[0]
	    diff_theta_gps = gps_euler[0] - last_gps_euler[0]

	    theta_error = diff_theta_odo - diff_theta_gps
	    pos_error = diff_gps - diff_odo

	    sum_error += pos_error.norm
	    wheels_diff = [wheels,wheels_old].transpose.map {|x| x.reduce(:-)}
	    os.puts "#{diff_gps.to_a.join(' ')} #{diff_odo.to_a.join(' ')} #{theta_error} #{euler.to_a.join(' ')} #{wheels_diff.min} #{wheels_diff.max} #{wheels_diff.mean}"
	    counter = 0
	else
	    counter += 1
	end

	last_gps = gps_pos
	last_odo = odo_pos
	last_orientation = orientation
	last_euler = euler
	last_gps_euler = gps_euler
	wheels_old = wheels
    end
end

os.close
