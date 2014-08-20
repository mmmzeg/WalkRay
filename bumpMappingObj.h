#include "abstractObj.h"
#include "bumpMappingBsdf.h"
#include "texturePlaneMap.h"
#include <algorithm>

namespace LFJ{
	class BumpMappingObject : public AbstractObject{
		TexturePlaneMap *t_map;
	public:
		BumpMappingObject(Scene *scene) : AbstractObject(scene){
			bsdf = new BumpMappingBSDF();
			t_map = NULL;
		}
		void setTexture(const std::string &diff, const std::string &norm){
			vec3f p_min,  p_max;
			getBoundingBox(p_min, p_max);
			t_map = new TexturePlaneMap(this->transform, p_min, p_max);
			t_map->use_normal_map = false;
			t_map->loadTextureMap(diff, norm);
		}
		void setTMapUseNormalMap(bool use){
			t_map->use_normal_map = use;
		}
		~BumpMappingObject(){
			delete bsdf;
			if(t_map)	delete t_map;
		}
		vec3f evaluateBSDF(const Ray &inRay, const Ray &outRay) const{
			if(!outRay.contactObj)	return vec3f(0,0,0);
			vec3f _normal = outRay.contactObj->getWorldNormal(outRay.contactTriangleID, outRay.origin);
			

			vec3f normal = t_map->getNormal(outRay.origin, _normal);
			

			vec3f diffuse = t_map->getDiffuse(outRay.origin);
			return diffuse * bsdf->evalBSDF(inRay.direction, outRay.direction, normal);
		}
		float evalDirectionProbability(const Ray &inRay, const Ray &outRay) const{
			if(!outRay.contactObj)	return 0;
			vec3f _normal = outRay.contactObj->getWorldNormal(outRay.contactTriangleID, outRay.origin);
			
			
			vec3f normal = t_map->getNormal(outRay.origin, _normal);
			

			//vec3f color = static_cast<DiffuseBSDF*>(bsdf)->reflectanceAlbedo;
			vec3f color = t_map->getDiffuse(outRay.origin);
			float p = *std::max_element<float*>(&color.x, &color.x + 3);
			return p * bsdf->evalPdfW(inRay.direction, outRay.direction, normal);
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

			//vec3f color = static_cast<DiffuseBSDF*>(bsdf)->reflectanceAlbedo;
			
			vec3f color = t_map->getDiffuse(position);
			vec3f normal = t_map->getNormal(position, worldNormal);

			float p = *std::max_element<float*>(&color.x, &color.x + 3);
			if(rng->genFloat() < p){
				outRay.radiance = color * bsdf->sampleBSDF(inRay.direction, outRay.direction, normal, *rng, &outRay.directionProb);
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