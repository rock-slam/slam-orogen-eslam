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
	    @config[:hash] = task.hash_config.dup 
	end

	def to_task task
	    task.eslam_config = @config[:eslam]
	    task.odometry_config = @config[:odometry] 
	    task.asguard_config = @config[:asguard] 
	    task.hash_config = @config[:hash] 
	end

	def pretty_print pp
	    @config.each do |c|
		pp c
	    end
	end

	def update_params config 
	    method( config ).call
	end	

	def localisation 
	    c = @config[:eslam]
	    c.seed = @seed
	    c.mappingThreshold.distance = 0.02
	    c.mappingThreshold.angle = 3.0 * Math::PI/180.0
	    c.measurementThreshold.distance = 0.05
	    c.measurementThreshold.angle = 5.0 * Math::PI/180.0
	    c.initialError = 0.5
	    c.particleCount = 250
	    c.minEffective = 50
	    c.measurementError = 0.15
	    c.discountFactor = 0.95
	    c.spreadThreshold = 0.5
	    c.spreadTranslationFactor = 0.0001
	    c.spreadRotationFactor = 0.00005
	    c.slipFactor = 0.2

	    c = @config[:odometry]
	    c.seed = @seed
	    c.constError.translation = Eigen::Vector3.new( 0.01, 0.01, 0.0 )
	    c.constError.yaw = 0.005 
	    c.distError.translation = Eigen::Vector3.new( 0.1, 0.5, 0.0 )
	    c.distError.yaw = 0.001 
	    c.tiltError.translation = Eigen::Vector3.new( 0.01, 0.01, 0.0 )
	    c.tiltError.yaw = 0.001
	    c.dthetaError.translation = Eigen::Vector3.new( 0.05, 0.01, 0.0 )
	    c.dthetaError.yaw = 0.005

	    c = @config[:hash]
	    c.useHash = true
	    c.period = 20
	    c.percentage = 0.05
	    c.avgFactor = 0.8
	    c.slopeBins = 20
	    c.angularSteps = 16 
	end

	def mapping 
	    c = @config[:eslam]
	    c.seed = @seed
	    c.mappingThreshold.distance = 0.05
	    c.mappingThreshold.angle = 3.0 * Math::PI/180.0
	    c.measurementThreshold.distance = 0.05
	    c.measurementThreshold.angle = 5.0 * Math::PI/180.0
	    c.initialError = 1.0
	    c.particleCount = 250
	    c.minEffective = 50
	    c.measurementError = 0.05
	    c.discountFactor = 0.95
	    c.spreadThreshold = 0.5
	    c.spreadTranslationFactor = 0.0001
	    c.spreadRotationFactor = 0.00005
	    c.slipFactor = 0.2

	    c = @config[:odometry]
	    c.seed = @seed
	    c.constError.translation = Eigen::Vector3.new( 0.0015, 0.0015, 0.0 )
	    c.constError.yaw = 5e-3 
	    c.distError.translation = Eigen::Vector3.new( 0.04, 0.10, 0.0 )
	    c.distError.yaw = 1e-4 
	    c.tiltError.translation = Eigen::Vector3.new( 0.01, 0.01, 0.0 )
	    c.tiltError.yaw = 0.001
	    c.dthetaError.translation = Eigen::Vector3.new( 0.01, 0.01, 0.0 )
	    c.dthetaError.yaw = 0.001
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
