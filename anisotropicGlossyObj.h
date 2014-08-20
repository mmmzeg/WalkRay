#include "abstractObj.h"
#include "anisotropicPhong.h"
#include <algorithm>
#include <omp.h>

namespace LFJ{
	class AnisotropicGlossyObject : public AbstractObject{
	public:
		float anisotropicRatio;
		float Nv, Nu;
	public:
		AnisotropicGlossyObject(Scene *scene) : AbstractObject(scene){
			bsdf = new AnisotropicPhong();
		}
		~AnisotropicGlossyObject(){
			delete bsdf;
		}
		void setRs_Rd_Nu_Nv(const vec3f &rs, const vec3f &rd, const float n_u, const float n_v){
			static_cast<AnisotropicPhong*>(bsdf)->Rs = rs;
			static_cast<AnisotropicPhong*>(bsdf)->Rd = rd;
			static_cast<AnisotropicPhong*>(bsdf)->n_u = n_u;
			static_cast<AnisotropicPhong*>(bsdf)->n_v = n_v;
			vec3f min_p = vec3f(this->transform * vec4f(this->minCoord, 1));
			vec3f max_p = vec3f(this->transform * vec4f(this->maxCoord, 1));
			float maxN = MAX(n_u,n_v), minN = MIN(n_u,n_v);
			anisotropicRatio = std::sqrtf(maxN / minN);
			std::cout << "Nu = " << n_u << " Nv = " << n_v  << " aniRatio = " << anisotropicRatio << std::endl;
			Nu = n_u, Nv = n_v;
		}
		void set_disk_frame(bool f){
		}
		BSDF *refBSDF() const { return this->bsdf; }
		bool anisotropic() const { return true; }
		bool nonSpecular() const { 
#ifdef MERGE_GLOSSY
			return true;
#else
			return false;
#endif
		}
		vec3f evaluateBSDF(const Ray &inRay, const Ray &outRay) const{
			if(!outRay.contactObj)	return vec3f(0,0,0);
			vec3f normal = outRay.contactObj->getWorldNormal(outRay.contactTriangleID, outRay.origin);
			vec3f bsdfV = bsdf->evalBSDF(inRay.direction, outRay.direction, normal);
			return bsdfV;
		}
		float evalDirectionProbability(const Ray &inRay, const Ray &outRay) const{
			if(!outRay.contactObj)	return 0;
			vec3f normal = outRay.contactObj->getWorldNormal(outRay.contactTriangleID, outRay.origin);
			float pdfV = bsdf->evalPdfW(inRay.direction, outRay.direction, normal);
			return pdfV;
		}
		 
		Ray scatter(Ray &inRay) const{
			Ray outRay;
			vec3f position = inRay.origin + inRay.direction * inRay.intersectDist;
			vec3f worldNormal = inRay.intersectObj->getWorldNormal(inRay.intersectTriangleID, position);
			
			outRay.origin = position;
			outRay.insideObj = inRay.insideObj;
			outRay.intersectObj = NULL;
			outRay.contactObj = (AbstractObject*)this;
			outRay.contactTriangleID = inRay.intersectTriangleID;

			outRay.radiance = bsdf->sampleBSDF(inRay.direction, outRay.direction, worldNormal, *rng, &outRay.directionProb);

			return outRay;
		}
	};
}
