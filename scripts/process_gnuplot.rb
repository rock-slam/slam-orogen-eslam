#! /usr/bin/env ruby
require 'eslam'
require 'plots'
require 'optparse'

opts = {}
p = OptionParser.new do |o|
    o.banner = "usage: process_gnuplot.rb [options] log_dir"
    o.on("-e", "--env=PATH", "Path to environment") {|v| opts[:env_dir] = v }
    o.on("-o", "--out=PATH", "Output directory (will be created)") {|v| opts[:out_dir] = v }
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
	@plot.save_pdf( File.join( output_dir, "position.pdf" ) )
    end

    def show()
	@plot.show
    end

end

eslam = GpEslam.new( opts[:log_dir], opts[:env_dir] )
eslam.run
if opts[:out_dir]
    eslam.save( opts[:out_dir] )
else
    eslam.show
end

