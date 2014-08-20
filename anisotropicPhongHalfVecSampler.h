#pragma once
#include "sampler.h"
#include "frame.h"

namespace LFJ{
	class AnisotropicPhongHalfVecSampler : public Sampler{
		float n_v, n_u;
		vec3f norm;
	public:
		AnisotropicPhongHalfVecSampler(float v, float u, const vec3f &n) : n_v(v), n_u(u), norm(n) {
		
		}
		/* http://www.cs.utah.edu/~michael/brdfs/jgtbrdf.pdf */
		vec3f sample(Rng &rng, const vec3f &localFix, float *oPdfW = NULL) const{
			const vec2f sample = rng.genVec2f();
			
			float phi = std::atan( std::sqrt((n_u + 1.0) / (n_v + 1.0)) * std::tan(PI/2.0*sample.x) );
			
			float choose = rng.genFloat();

			if(choose >= 0 && choose < 0.25)			/* nothing */;
			else if(choose >= 0.25 && choose < 0.5)		phi = PI - phi;  
			else if(choose >= 0.5 && choose < 0.75)		phi += PI;
			else										phi = 2*PI - phi; 


			const float cosTheta = std::powf((1.0 - sample.y), 1.0/(1.0+n_u*cos(phi)*cos(phi)+n_v*sin(phi)*sin(phi))); 
			const float sinTheta = std::sqrt(1.0 - cosTheta*cosTheta);

			const vec3f ret(
				sinTheta * sin(phi),
				cosTheta,
				sinTheta * cos(phi)
			);

			return ret;
		}

		/* \brief: calculate p(h), the half vector's probability density. */
		float pdf(const vec3f &localFix, const vec3f &halfVector) const{ 
			Frame frame;

			frame.setFromN(norm);
			

			vec3f localHalfVec = frame.toLocalFrame(halfVector);
			
			const float cosTheta = localHalfVec.y;
			const float sinTheta = std::sqrt(1.0 - cosTheta*cosTheta);
			if(sinTheta < EPSILON)
				return 0;
			const float cosPhi = localHalfVec.z / sinTheta;
			const float sinPhi = localHalfVec.x / sinTheta;

			float dotValue = norm.dot(halfVector);
			return std::sqrt((n_u+1.0)*(n_v+1.0)) / (2.0*PI) * std::powf(dotValue, n_u*cosPhi*cosPhi + n_v*sinPhi*sinPhi);
		}

	};
}