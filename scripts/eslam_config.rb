module Eslam
    class Config
	def initialize seed = 42
	    @seed = seed
	    @config = {}
	end

	def update task, config = :single
	    from_task task
	    update_params config
	    to_task task
	end

	def from_task task
	    @config[:eslam] = task.eslam_config.dup
	    @config[:odometry] = task.odometry_config.dup
	    @config[:asguard] = task.asguard_config.dup 
	end

	def to_task task
	    task.eslam_config = @config[:eslam]
	    task.odometry_config = @config[:odometry] 
	    task.asguard_config = @config[:asguard] 
	end

	def pretty_print pp
	    @config.each do |c|
		pp c
	    end
	end

	def update_params config 
	    method( config ).call
	end	

	def mapping 
	    @config[:eslam].instance_exec do
		seed = @seed
		mappingThreshold.distance = 0.02
		mappingThreshold.angle = 3.0 * Math::PI/180.0
		measurementThreshold.distance = 0.01
		measurementThreshold.angle = 3.0 * Math::PI/180.0
		initialError = 0.0
		particleCount = 250
		minEffective = 50
		measurementError = 0.05
		discountFactor = 0.95
		spreadThreshold = 0.5
		spreadTranslationFactor = 0.0001
		spreadRotationFactor = 0.00005
		slipFactor = 0.2
	    end

	    @config[:odometry].instance_exec do
		seed = @seed
		constError.translation = Eigen::Vector3.new( 0.0005, 0.0005, 0.0 )
		constError.yaw = 0e-5 

		distError.translation = Eigen::Vector3.new( 0.04, 0.10, 0.0 )
		distError.yaw = 1e-4 

		tiltError.translation = Eigen::Vector3.new( 0.01, 0.01, 0.0 )
		tiltError.yaw = 0.001

		dthetaError.translation = Eigen::Vector3.new( 0.01, 0.01, 0.0 )
		dthetaError.yaw = 0.001
	    end

	    @config[:asguard].instance_exec do
	    end
	end

	def single 
	    c = @config[:eslam]
	    c.seed = @seed
	    c.mappingThreshold.distance = 0.02
	    c.mappingThreshold.angle = 3.0 * Math::PI/180.0
	    c.measurementThreshold.distance = 1000.0
	    c.measurementThreshold.angle = 3000.0 * Math::PI/180.0
	    c.initialError = 0.0
	    c.particleCount = 1
	    c.minEffective = 1 
	    c.measurementError = 0.05
	    c.discountFactor = 0.95
	    c.spreadThreshold = 0.5
	    c.spreadTranslationFactor = 0.0001
	    c.spreadRotationFactor = 0.00005
	    c.slipFactor = 0.2

	    c = @config[:odometry]
	    c.seed = @seed
	    c.constError.translation = Eigen::Vector3.new( 0.0000, 0.0000, 0.0 )
	    c.constError.yaw = 0e-5 
	    c.distError.translation = Eigen::Vector3.new( 0.00, 0.00, 0.0 )
	    c.distError.yaw = 0e-4 
	    c.tiltError.translation = Eigen::Vector3.new( 0.00, 0.00, 0.0 )
	    c.tiltError.yaw = 0.000
	    c.dthetaError.translation = Eigen::Vector3.new( 0.00, 0.00, 0.0 )
	    c.dthetaError.yaw = 0.000
	end
    end
end
