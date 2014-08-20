#include "abstractObj.h"
#include "diffuseBsdf.h"
#include <algorithm>

namespace LFJ{
	class DiffuseObject : public AbstractObject{
	public:
		DiffuseObject(Scene *scene) : AbstractObject(scene){
			bsdf = new DiffuseBSDF();
		}
		~DiffuseObject(){
			delete bsdf;
		}
		void setColor(vec3f &color){
			static_cast<DiffuseBSDF*>(bsdf)->reflectanceAlbedo = color;
		}
		vec3f evaluateBSDF(const Ray &inRay, const Ray &outRay) const{
			if(!outRay.contactObj)	return vec3f(0,0,0);
			vec3f normal = outRay.contactObj->getWorldNormal(outRay.contactTriangleID, outRay.origin);
			return bsdf->evalBSDF(inRay.direction, outRay.direction, normal);
		}
		float evalDirectionProbability(const Ray &inRay, const Ray &outRay) const{
			if(!outRay.contactObj)	return 0;
			vec3f normal = outRay.contactObj->getWorldNormal(outRay.contactTriangleID, outRay.origin);
			vec3f color = static_cast<DiffuseBSDF*>(bsdf)->reflectanceAlbedo;
			float p = *std::max_element<float*>(&color.x, &color.x + 3);
			return p * bsdf->evalPdfW(inRay.direction, outRay.direction, normal);
		}
		float continueProbability() const{
			vec3f &color = static_cast<DiffuseBSDF*>(bsdf)->reflectanceAlbedo;
			float p = *std::max_element<float*>(&color.x, &color.x + 3);
			return p;
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

			vec3f color = static_cast<DiffuseBSDF*>(bsdf)->reflectanceAlbedo;
			float p = *std::max_element<float*>(&color.x, &color.x + 3);
			if(rng->genFloat() < p){
				outRay.radiance = bsdf->sampleBSDF(inRay.direction, outRay.direction, worldNormal, *rng, &outRay.directionProb);
				outRay.directionProb *= p;
			}
			else{
				outRay.direction = vec3f(0,0,0);
				outRay.radiance = vec3f(0,0,0);
				outRay.directionProb = 1 - p;
			}	
			return outRay;
		}
	};
}