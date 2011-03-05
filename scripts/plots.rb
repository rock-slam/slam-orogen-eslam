require 'gnuplot'
#
# plot = PositionPlot.new('test')
# plot.register( :centroid, {:title => "Particle Filter"} )
# plot.register( :gps, {:title => "GPS", :lt => -1} )
# plot.register( :odometry, {:title => "Odometry"} )
# 
# # now do multiples of
# plot.data( :centroid, row )
# plot.data( :odometry, row )
# plot.data( :gps, [row[0],row[1]], [row[3],row[4]] )
# 
# # you can either save, or show the plot
# plot.save( ARGV[1] )
# 

class PositionPlot
    def initialize( title )
	@title = title

	@plots = Hash.new
    end

    def register( sym, params )
	@plots[sym] = Hash.new
	@plots[sym][:pos] = [[],[]]
	@plots[sym][:error] = [[],[]]
	@plots[sym].merge!( params )
    end

    def data( sym, pos, error = nil )
	@plots[sym][:pos][0] << pos[0]
	@plots[sym][:pos][1] << pos[1]
	if error
	    @plots[sym][:error][0] << error[0]
	    @plots[sym][:error][1] << error[1]
	end
    end

    def save( file_name )
	File.open( file_name, 'w') do |file|
	    plot( file )
	end
    end
    
    def show
	Gnuplot.open do |gp|
	    plot( gp )
	end
    end

    def plot( gp )
	Gnuplot::Plot.new( gp ) do |plot|

	    plot.set "title '#{@title}' font 'Helvetica,20'"
	    plot.set "xlabel 'x (East)(m)' font 'Helvetica,14'"
	    plot.set "ylabel 'y (North)(m)' font 'Helvetica,14'"
	    plot.set "key spacing 1.5"
	    plot.set "key font 'Helvetica,14'"
	    plot.set "key right bottom"

	      
	    #plot.arbitrary_lines  "set ylabel \"y label" font \"Helvetica,20\"" 
	    
	    plots = Array.new
	    obj_idx = 1

	    @plots.each_value do |p|
		p[:error][0].length.times do |i|
		    plot.object  "#{obj_idx} ellipse center #{p[:pos][0][i]}, #{p[:pos][1][i]} size #{p[:error][0][i]},#{p[:error][1][i]}  fs empty border #{p[:lt]}" 
		    obj_idx += 1
		end

		if p[:pos][0].length > 0
		    plots << Gnuplot::DataSet.new( p[:pos] ) { |ds|
			ds.with = "linespoints"
			ds.title = p[:title] 
			if p[:lt]
			    ds.with = "l lt #{p[:lt]}"
			end
		    }
		end
	    end

	    plot.data = plots
	end
    end
end

