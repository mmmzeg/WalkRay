#pragma once
#include "bsdf.h"
#include "util.h"
#include "hgPhaseSampler.h"

namespace LFJ{
	class PhaseFunction : public BSDF{
		HGPhaseSampler hgPhaseSampler;
		float g;
	public:
		PhaseFunction(float g) : hgPhaseSampler(g), g(g) {}
	public:
		/*\brief: evaluate pdf with respect to solid angle(W) */
		float evalPdfW(const vec3f &inRay, const vec3f &outRay, const vec3f &normal) const{
			return hgPhaseSampler.pdf(inRay, outRay);
		}

		/*\brief: sample a Gen direction with respect to BSDF, return BSDF vec3f */
		vec3f sampleBSDF(const vec3f &inRay, vec3f &outRay, const vec3f &normal, Rng &rng, float *oPdfW = NULL) const{
			outRay = hgPhaseSampler.sample(rng, inRay, oPdfW);
			float cosTheta = inRay.dot(outRay);
			float temp = 1 + g*g - 2*g*cosTheta;

			return 1/(4*PI) * (1-g*g) / (temp*sqrt(temp));
		}

		/*\brief: evaluate the BSDF vec3f given Fix direction and Gen direction */
		vec3f evalBSDF(const vec3f &inRay, const vec3f &outRay, const vec3f &normal) const{
			float cosTheta = inRay.dot(outRay);
			float temp = 1 + g*g - 2*g*cosTheta;

			return 1/(4*PI) * (1-g*g) / (temp*sqrt(temp));
		}
	};
}
