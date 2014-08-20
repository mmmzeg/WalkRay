#pragma once
#include "bsdf.h"
#include "util.h"
#include "anisotropicPhongHalfVecSampler.h"

namespace LFJ{
	class AnisotropicPhong : public BSDF{
	public:
		vec3f Rs, Rd; 
		float n_u, n_v;
	public:
		AnisotropicPhong(){
		}
		AnisotropicPhong(const vec3f &rs, const vec3f &rd, const float nu, const float nv):
			Rs(rs), Rd(rd), n_u(nu), n_v(nv) 
		{
		}
		/*\brief: evaluate pdf with respect to solid angle(W) */
		float evalPdfW(const vec3f &inRay, const vec3f &outRay, const vec3f &normal) const{
			if(inRay.dot(normal) <= 0 && outRay.dot(normal) >= 0){
				Frame localFrame;
				localFrame.setFromN(normal);

				AnisotropicPhongHalfVecSampler anisotropicPhongHalfVecSampler(n_v, n_u, normal);
				/* half vector: http://en.wikipedia.org/wiki/Blinn%E2%80%93Phong_shading_model */
				vec3f halfVector = outRay - inRay;
				halfVector.normalize();
				float oPdfW = anisotropicPhongHalfVecSampler.pdf(localFrame.toLocalFrame(inRay), halfVector);
				oPdfW /= 4 * std::fabsf(inRay.dot(halfVector));
				return oPdfW;
			}
			return 0.0;
		}

		/*\brief: sample a Gen direction with respect to BSDF, return BSDF vec3f */
		// NOTE: diffuse term sampling is not implemented yet, which should be added here with MIS.
		vec3f sampleBSDF(const vec3f &inRay, vec3f &outRay, const vec3f &normal, Rng &rng, float *oPdfW = NULL) const{
			Frame localFrame;
			localFrame.setFromN(normal);

			AnisotropicPhongHalfVecSampler anisotropicPhongHalfVecSampler(n_v, n_u, normal);
			const vec3f halfVector = localFrame.toWorldFrame(anisotropicPhongHalfVecSampler.sample(rng, vec3f()));
			outRay = inRay + 2*halfVector*(-inRay.dot(halfVector));
			if(oPdfW){
				*oPdfW = normal.dot(outRay) > 0 ? 
					anisotropicPhongHalfVecSampler.pdf(localFrame.toLocalFrame(inRay), halfVector) : 0;
				*oPdfW /= 4 * std::fabsf(inRay.dot(halfVector));
			}
			if(outRay.dot(normal) < 0){
				outRay = vec3f(0,0,0);
				return 0.0;
			}
			return evalBSDF(inRay, outRay, normal);
		}

		/*\brief: evaluate the BSDF vec3f given Fix direction and Gen direction */
		vec3f evalBSDF(const vec3f &inRay, const vec3f &outRay, const vec3f &normal) const{
			if(inRay.dot(normal) <= 0 && outRay.dot(normal) >= 0){
				vec3f specularComp = 0.0, diffuseComp = 0.0;
				vec3f halfVector = outRay - inRay;	halfVector.normalize();
				Frame localFrame;	

				localFrame.setFromN(normal);

				// specular: http://www.cs.utah.edu/~michael/brdfs/jgtbrdf.pdf (2) 
				const double n_dot_h = normal.dot(halfVector);
				const double k_dot_h = std::fabsf(inRay.dot(halfVector));
				const vec3f SchlickApproximationToFresnelFraction = 
					Rs + (vec3f(1,1,1) - Rs) * std::powf(1.0-k_dot_h, 5);


				const double hu = halfVector.dot(localFrame.mZ);
				const double hv = halfVector.dot(localFrame.mX);

				const float exponentValue = 
					fabs(n_dot_h) < EPSILON ? 
					1 : 
					(n_u * std::powf(hu, 2) +
					n_v * std::powf(hv, 2)
					) / (1.0 - std::powf(n_dot_h, 2));


				const double ASG = fabs(n_dot_h) * exp((double)(-n_u/2*hu*hu-n_v/2*hv*hv));

				specularComp = std::sqrt((n_u+1.0)*(n_v+1.0)) / (8*PI) * SchlickApproximationToFresnelFraction
					* std::powf(n_dot_h, exponentValue) /* ASG *// (k_dot_h * MAX(std::fabsf(normal.dot(inRay)), std::fabsf(normal.dot(outRay/*inRay*/))));

				// diffuse: http://www.cs.utah.edu/~michael/brdfs/jgtbrdf.pdf (5)
				const float term1 = 1.0 - (-inRay.dot(normal)/2);
				const float term2 = 1.0 - (outRay.dot(normal)/2);
				diffuseComp = 28*Rd / (23*PI) * (vec3f(1,1,1)-Rs)
					* (1 -   std::powf(term1,5)     )
					* (1 -   std::powf(term2,5)     );

				/*std::cout << "Sbsdf = " << specularComp << std::endl;
				std::cout << "Dbsdf = " << diffuseComp << std::endl;*/
				//return ASG;

				return specularComp + diffuseComp;
			}
			return vec3f(0,0,0);
		}

	};
}
