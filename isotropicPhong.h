#pragma once
#include "bsdf.h"
#include "util.h"
#include "powerCosHemiSphereSampler.h"

namespace LFJ{
	class IsotropicPhong : public BSDF{
	public:
		vec3f reflectanceAlbedo; 
		float power;
	public:
		/*\brief: evaluate pdf with respect to solid angle(W) */
		float evalPdfW(const vec3f &inRay, const vec3f &outRay, const vec3f &normal) const{
			const float cosTheta = MAX(normal.dot(outRay), EPSILON);
			
			return (power + 1.0) * std::pow(cosTheta, power) * (INV_PI * 0.5);
		}

		/*\brief: sample a Gen direction with respect to BSDF, return BSDF vec3f */
		vec3f sampleBSDF(const vec3f &inRay, vec3f &outRay, const vec3f &normal, Rng &rng, float *oPdfW = NULL) const{
			Frame localFrame;
			localFrame.setFromN(normal);
			PowerCosHemiSphereSampler cosSampler(power);
			outRay = localFrame.toWorldFrame(cosSampler.sample(rng, vec3f()));
			if(oPdfW)
				*oPdfW = normal.dot(outRay) > 0 ? 
						(power+1.0)*std::pow(normal.dot(outRay), power)*(INV_PI*0.5) : 0;

			vec3f fixReflDir = -normal.dot(inRay)*normal*2 + inRay; 	
			fixReflDir.normalize();
			const vec3f rho = reflectanceAlbedo * (power + 2.0) * 0.5 * INV_PI;
			const float dot_R_Wi = fixReflDir.dot(outRay);
			if(outRay.dot(normal) < 0){
				outRay = vec3f(0,0,0);
				return EPSILON;
			}
			return rho * std::pow(MAX(dot_R_Wi,0), power);
		}

		/*\brief: evaluate the BSDF vec3f given Fix direction and Gen direction */
		vec3f evalBSDF(const vec3f &inRay, const vec3f &outRay, const vec3f &normal) const{
			if(inRay.dot(normal) <= 0 && outRay.dot(normal) >= 0){
				vec3f fixReflDir = -normal.dot(inRay)*normal*2 + inRay; 	
				fixReflDir.normalize();
				const vec3f rho = reflectanceAlbedo * (power + 2.0) * 0.5 * INV_PI;
				const float dot_R_Wi = fixReflDir.dot(outRay);
				return rho * std::pow(MAX(dot_R_Wi,0), power);
			}
			return vec3f(0,0,0);
		}
	};
}
