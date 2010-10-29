#ifndef __ESLAM_PARTICLEWRAPPER_HPP_
#define __ESLAM_PARTICLEWRAPPER_HPP_

#include <base/wrappers/eigen.h>
#include <base/wrappers/pose.h>

#ifndef __orogen
#include <particle_filter/PoseEstimator.hpp>
#endif

namespace wrappers
{
struct ContactPoint
{
    Vector3 point;
    double z_diff;
    double z_var;

#ifndef __orogen
    ContactPoint() {}
    ContactPoint( const eslam::ContactPoint& cp )
	: point( cp.point ), z_diff( cp.zdiff ), z_var( cp.zvar ) {}

    ContactPoint& operator=( const eslam::ContactPoint& cp )
    {
	point = cp.point;
	z_diff = cp.zdiff;
	z_var = cp.zvar;
	return *this;
    }
    
    operator eslam::ContactPoint() const 
    {
	return eslam::ContactPoint( point, z_diff, z_var );
    }
#endif
};

struct PoseParticle
{
    double x, y;
    double theta;
    double zPos;
    double zSigma;
    bool floating;
    std::vector<ContactPoint> cpoints;
    wrappers::Vector3 meas_pos;
    double meas_theta;
    double weight;

#ifndef __orogen
    typedef eslam::ParticleFilter<eslam::PoseParticle>::Particle particle;

    PoseParticle() {}
    PoseParticle( const particle& p )
	: x( p.x.position.x() ), y( p.x.position.y() ), theta( p.x.orientation ),
	zPos( p.x.zPos ), zSigma( p.x.zSigma ), floating( p.x.floating ), 
	cpoints(p.x.cpoints.begin(), p.x.cpoints.end()), meas_pos( p.x.meas_pos ), meas_theta( p.x.meas_theta ),
	weight( p.w )
    {}

    PoseParticle& operator=( const particle& p )
    {
	x = p.x.position.x();
	y = p.x.position.y();
	theta = p.x.orientation;
	zPos = p.x.zPos;
	zSigma = p.x.zSigma;
	floating = p.x.floating;
	std::copy( p.x.cpoints.begin(), p.x.cpoints.end(), std::back_inserter( cpoints ) );
	meas_pos = p.x.meas_pos;
	meas_theta = p.x.meas_theta;
	weight = p.w;

	return *this;
    }

    operator particle() const
    {
	eslam::PoseParticle pp( Eigen::Vector2d( x, y ), theta, zPos, zSigma, floating );
	std::copy( cpoints.begin(), cpoints.end(), std::back_inserter( pp.cpoints ) );
	pp.meas_pos = meas_pos;
	pp.meas_theta = meas_theta;

	return particle( pp, weight);
    }
#endif
};

struct PoseDistribution
{
    base::Time time;
    std::vector<PoseParticle> particles;
    wrappers::Quaternion orientation;
    wrappers::BodyState body_state;

#ifndef __orogen
    PoseDistribution() {}
    PoseDistribution( const eslam::PoseDistribution& p )
	: time( p.time ), particles( p.particles.begin(), p.particles.end() ), orientation( p.orientation ), body_state( p.bodyState )
    {}

    operator eslam::PoseDistribution() const
    {
	eslam::PoseDistribution result;
	result.time = time;
	for( std::vector<PoseParticle>::const_iterator it=particles.begin();it!=particles.end();it++ )
	{
	    result.particles.push_back( *it );
	}
	result.orientation = orientation;
	result.bodyState = body_state;

	return result;
    }

#endif
};

}
#endif
