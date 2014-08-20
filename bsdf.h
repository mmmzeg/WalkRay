#pragma once
#include "rng.h" 
#include "frame.h"

namespace LFJ{
	class BSDF{
	public:
		/*\brief: evaluate pdf with respect to solid angle(W) */
		virtual float evalPdfW(
			const vec3f &inRay,
			const vec3f &outRay,
			const vec3f &normal) const = 0;

		/*\brief: sample a Gen direction with respect to BSDF, return BSDF vec3f */
		virtual vec3f sampleBSDF(
			const vec3f &inRay, 
			vec3f	    &outRay,
			const vec3f &normal,
			Rng		    &rng,
			float	    *oPdfW = NULL) const = 0;    

		/*\brief: evaluate the BSDF vec3f given Fix direction and Gen direction */
		virtual vec3f evalBSDF(
			const vec3f &inRay, 
			const vec3f &outRay, 
			const vec3f &normal) const = 0;
	};
}