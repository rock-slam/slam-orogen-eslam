
class SequenceArray
    class Sequence
	def initialize( samples = [], times = [] )
	    @idx = 0
	    @samples = samples 
	    @times = times 
	end

	def clone 
	    Sequence.new @idx.clone, @samples.clone
	end

	def push sample, ts
	    @samples << sample
	    @times << ts
	end

	def current_time
	    @times[@idx] if @idx < @samples.size
	end

	def current_time=( val )
	    @times[@idx] = val
	end

	def next_time
	    @times[@idx+1] if (@idx+1) < @samples.size
	end

	def current
	    @samples[@idx]
	end

	def current=( val )
	    @samples[@idx] = val
	end

	def advance
	    @idx += 1 unless @idx >= @samples.size
	    current_time
	end

	attr_accessor :idx, :samples, :times
    end

    def initialize( seq = {} )
	@seq = seq 
    end

    def clone
	SequenceArray.new @seq.clone
    end

    def add( name )
	@seq[name] = Sequence.new
    end

    def advance( name )
	if t = @seq[name].advance
	    @seq.each do |sn, sv|
		while sv.next_time and sv.next_time < t do sv.advance end
	    end
	end
    end

    def each( name )
	reset
	return unless @seq[name].current
	    
	begin
	    yield self
	end while advance(name)
    end

    def reset
	@seq.each do |sn, sv|
	    sv.idx = 0
	end
    end

    attr_accessor :seq
end

