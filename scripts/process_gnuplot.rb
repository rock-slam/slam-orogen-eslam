#! /usr/bin/env ruby
require 'eslam'
require 'plots'

if ARGV.size < 1 then 
    puts "usage: process_logs.rb log_dir [environment_path] [output_dir]"
    exit
end

class GpEslam < Eslam
    def configure
	super

	# set up the plotting class
	@plot = PositionPlot.new("Position Plot")
	@plot.register( :gps, {:title => "gps"} )
	@plot.register( :odometry, {:title => "odometry"} )
	@plot.register( :eslam, {:title => "eslam"} )

	@eslam.pose_samples.connect_to :type => :buffer,:size => 100 do |data, name| 
	    @plot.data( :eslam, data.position ) if data
	    data
	end

	@log_replay.mb500.position_samples.connect_to :type => :buffer,:size => 100 do |data, name|
	    @plot.data( :gps, data.position ) if data
	    data
	end

	@log_replay.odometry.odometry_samples.connect_to :type => :buffer,:size => 100 do |data, name|
	    if data
		@odometry_offset ||= @start_pos.position - data.position 
		odometry_pos = data.position + @odometry_offset
		@plot.data( :odometry, odometry_pos)
	    end
	    data
	end
    end

    def start
	#@log_replay.reset_time_sync
	#while @log_replay.step(true) do end
	Vizkit.control @log_replay
	Vizkit.exec
    end


    def save( output_dir )
	# create target directory
	Dir.mkdir( output_dir ) unless File.exist?( output_dir )
	@plot.save( File.join( output_dir, "position.gpl" ) )
    end

    def show()
	@plot.show
    end

end

eslam = GpEslam.new( ARGV[0], ARGV[1] )
eslam.run
#eslam.save( ARGV[2] )
eslam.show

