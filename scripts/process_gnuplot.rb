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
    def start
	#@log_replay.reset_time_sync
	#while @log_replay.step(true) do end
	Vizkit.control @log_replay
	Vizkit.exec
    end

    def prepare_plot
	return if @plot

	# set up the plotting class
	@plot = PositionPlot.new("Position Plot")
	@plot.register( :gps, {:title => "gps"} )
	@plot.register( :odometry, {:title => "odometry"} )
	@plot.register( :centroid, {:title => "eslam"} )

	@result.each(:gps) do |r|
	    [:centroid, :gps, :odometry].each do |s| 
		@plot.data( s, r.seq[s].current) if r.seq[s].current 
	    end
	end
    end

    def save( output_dir )
	prepare_plot
	# create target directory
	Dir.mkdir( output_dir ) unless File.exist?( output_dir )
	@plot.save( File.join( output_dir, "position.gpl" ) )
	@plot.save_pdf( File.join( output_dir, "position.pdf" ) )
	File.open( File.join( output_dir, "config.txt" ), 'w' ) do |file|
	    info( file )
	end
    end

    def show()
	prepare_plot
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

