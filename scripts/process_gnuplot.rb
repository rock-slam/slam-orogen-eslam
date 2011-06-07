#! /usr/bin/env ruby
require 'eslam'
require 'plots'
require 'optparse'

opts = {}
p = OptionParser.new do |o|
    o.banner = "usage: process_gnuplot.rb [options] log_dir"
    o.on("-e", "--env=PATH", "Path to environment") {|v| opts[:env_dir] = v }
    o.on("-o", "--out=PATH", "Output directory (will be created)") {|v| opts[:out_dir] = v }
    o.on("-d", "--debug_viz", "Use visualization for debugging") {|v| opts[:debug] = v }
    o.on("-i", "--interactive", "Use interactive controlls for replay") {|v| opts[:interactive] = v }
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
	if @opts[:interactive]
	    Vizkit.control @replay.log
	    Vizkit.exec
	else
	    super
	end
    end

    def configure
	super 
	@eslam.debug_viz = true if @opts[:debug]
    end

    def prepare_plot
	return if @plot

	# set up the plotting class
	@plot = PositionPlot.new("Position Plot")
	@plot.register( :gps, {:title => "gps", :lt => 1} )
	@plot.register( :odometry, {:title => "odometry", :lt => 2} )
	@plot.register( :centroid, {:title => "filter centroid", :lt => 3} )

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
	File.open( File.join( output_dir, "raw.dat" ), 'w' ) do |file|
	    @result.each(:gps) do |r|
		[:centroid, :gps, :odometry].each do |s| 
		    file.print r.seq[s].current.to_a.join(' ') if r.seq[s].current 
		    file.print " "
		end
		file.puts
	    end
	end
    end

    def show()
	prepare_plot
	@plot.show
    end

end

eslam = GpEslam.new( opts )
eslam.run
if opts[:out_dir]
    eslam.save( opts[:out_dir] )
else
    eslam.show
end

