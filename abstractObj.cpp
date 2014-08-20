#include "stdafx.h"
#include "abstractObj.h"
#include "uniformSphereSampler.h"
#include "scene.h"
#include "noself.h"
#include <algorithm>

namespace LFJ{
	void AbstractObject::prepareForSurfaceSampling(){
		float areaValue = 0;
		for(uint i = 0; i < getTriangleNum(); i++){
			areaValue += getTriangleArea(i);
			areaThreshold.push_back(areaValue);
		}
		boundArea = areaValue;
	}

	Ray AbstractObject::genSurfaceSample(){
		Ray ray;
		if(!areaThreshold.size()){
			ray.direction = vec3f(0,0,0);
			ray.radiance = vec3f(0,0,0);
			return ray;
		}

		float randArea = rng->genFloat() * boundArea;
		uint index = std::lower_bound(areaThreshold.begin(), areaThreshold.end(), randArea) - areaThreshold.begin();
		if(index == areaThreshold.size())	index--;
		
		ray.contactObj = this;
		ray.contactTriangleID = index;
		ray.origin = genRandTrianglePosition(ray.contactTriangleID);
		UniformSphereSampler uniformSampler;
		vec3f normal = ray.contactObj->getWorldNormal(ray.contactTriangleID, ray.origin);
		ray.direction = uniformSampler.sample(*rng, vec3f(), &ray.directionProb);
		if(ray.direction.dot(normal) < 0)	
			ray.direction = -ray.direction;
		ray.radiance = ray.evalBSDF(ray);
		ray.insideObj = scene->findInsideObject(ray, ray.contactObj);
		ray.directionProb *= 2;
		ray.originProb = 1 / boundArea * pickProb;

		Scene::ObjSourceInfo info;
		NoSelfCondition condition(scene, ray);
		float dist = scene->intersect(ray, info, &condition);
		if(dist > 0){
			ray.intersectObj = scene->objects[info.objID];
			ray.intersectDist = dist;
			ray.intersectTriangleID = info.triID;
		}
		return ray;
	}

	float AbstractObject::evalOriginProbability(const Ray &inRay, const Ray &outRay) const{
		if(&inRay == &outRay){ // emitted 
			return pickProb / boundArea;
		}
		return 1;
	}

	float AbstractObject::evalDirectionProbability(const Ray &inRay, const Ray &outRay) const{
		if(&inRay == &outRay){ // emitted 
			UniformSphereSampler sampler;
			vec3f worldNormal = outRay.contactObj->getWorldNormal(outRay.contactTriangleID, outRay.origin);
			return sampler.pdf(inRay.direction, outRay.direction) * 2;
		}
		return 0;
	}
		
	void AbstractObject::updateTransform(){
		if(!hasMotionBlur())	return ;

		float time = rng->genFloat();
		animatedTransform->Interpolate(time, &transform);
		return ;
	}
}