#pragma once
#include "bsdf.h"
#include "util.h"
#include "cosHemiSphereSampler.h"

namespace LFJ{
	class DiffuseBSDF : public BSDF{
	public:
		vec3f reflectanceAlbedo;
	public:
		/*\brief: evaluate pdf with respect to solid angle(W) */
		float evalPdfW(const vec3f &inRay, const vec3f &outRay, const vec3f &normal) const{
			return normal.dot(outRay) > 0 ? normal.dot(outRay) * INV_PI : 0;
		}

		/*\brief: sample a Gen direction with respect to BSDF, return BSDF vec3f */
		vec3f sampleBSDF(const vec3f &inRay, vec3f &outRay, const vec3f &normal, Rng &rng, float *oPdfW = NULL) const{
			Frame localFrame;
			localFrame.setFromN(normal);
			CosHemiSphereSampler cosSampler;
			outRay = localFrame.toWorldFrame(cosSampler.sample(rng, vec3f()));
			if(oPdfW)
				*oPdfW = normal.dot(outRay) > 0 ? normal.dot(outRay) * INV_PI : 0;
			
			return reflectanceAlbedo * INV_PI;
		}

		/*\brief: evaluate the BSDF vec3f given Fix direction and Gen direction */
		vec3f evalBSDF(const vec3f &inRay, const vec3f &outRay, const vec3f &normal) const{
			if(inRay.dot(normal) <= 0 && outRay.dot(normal) >= 0){
				return reflectanceAlbedo * INV_PI;
			}
			return vec3f(0,0,0);
		}
	};
}
