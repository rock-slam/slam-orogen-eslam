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
    o.on("-p", "--param=PARAM", "parameter to run batch on") {|v| opts[:param] = v }
    o.on("-r", "--range=RANGE", "range for parameter") {|v| opts[:range] = v }
    o.on("-s", "--step=STEP", "step size for range parameter") {|v| opts[:step] = v }
    o.on("-c", "--count=COUNT", "number of repititions per step") {|v| opts[:count] = v }
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

class Array
    def sum; inject { |sum,x| sum+x }; end
    def mean; sum / size; end; 
    def stdev; Math.sqrt( inject(0) { |sum,x| sum+(x-mean)**2 } / (size-1) ); end
end

module Eslam
class Batch < Replay 
    def params
	super 

	a = @opts[:param].split('.')
	param = "@configs[:#{a.shift}]"
	a.each {|v| param << ".#{v}"}

	puts("#{param} = @param_value")
	eval("#{param} = @param_value")
    end

    def process
	pp @opts
	exit unless @opts[:param] and @opts[:range] and @opts[:step]
	range = eval(@opts[:range])

	# create target directory
	Dir.mkdir( @opts[:out_dir] ) unless File.exist?( @opts[:out_dir] )

	File.open( File.join( @opts[:out_dir], "info.txt" ), 'w' ) do |file|
	    prev = $>
	    $> = file
	    pp @opts
	    $> = prev
	end

	File.open( File.join( @opts[:out_dir], "result.txt" ), 'w' ) do |file|
	    range.step( @opts[:step].to_f ) do |v|
		error = {:odometry => [], :centroid => []} 
		count = []
		1.upto( @opts[:count].to_i ) do |c|
		    @param_value = v
		    puts "running #{@opts[:param]} with value #{v}"
		    @seed = c
		    run

		    File.open( File.join( @opts[:out_dir], "debug_#{v}_#{c}.txt"), 'w') do |df|
			@result.each(:gps) do |r|
			    [:centroid, :gps, :odometry].each do |s| 
				df.print r.seq[s].current.to_a.join(' ') + ", " if r.seq[s].current
			    end
			    df.puts
			end
		    end

		    [:odometry, :centroid].each {|s| error[s] << @errors[s][:mean_error]}
		    count << @errors[:centroid][:count]
		end
		save
		file.puts "#{@param_value} #{error[:odometry].mean} #{error[:centroid].mean} #{error[:centroid].stdev} #{count.join(',')}"
	    end
	end
    end

    def save
	File.open( File.join( @opts[:out_dir], "config_#{@param_value}.txt" ), 'w' ) do |file|
	    info( file )
	end
    end
end
end

eslam = Eslam::Batch.new( opts )
eslam.process
