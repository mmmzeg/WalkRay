#pragma once
#include "mesh.h"
#include "ray.h"
#include "bsdf.h"
#include "motionBlur.h"

namespace LFJ{
	class Scene;

	class AbstractObject : public Mesh{
	protected:
		Scene *scene;
		BSDF  *bsdf;
		
	public:
		float pickProb;
		float boundArea;
		std::vector<float> areaThreshold;
		std::string name;

	public:
		AbstractObject(Scene *scene){
			this->scene = scene;
		//	this->unitize();
			bsdf = NULL;
			animatedTransform = NULL;
		}
		virtual Ray scatter(Ray &inRay) const { return inRay; }		
	public:
		virtual vec3f evaluateBSDF(const Ray &inRay, const Ray &outRay) const { return vec3f(1,1,1); }
		virtual float evalOriginProbability(const Ray &inRay, const Ray &outRay) const;
		virtual float evalDirectionProbability(const Ray &inRay, const Ray &outRay) const;
		virtual float continueProbability() const { return 1; }
		virtual void prepareForSurfaceSampling();
		virtual Ray genSurfaceSample();
		virtual float getIOR() const { return 1.0; }
		virtual bool isVolume() const { return false; }
		virtual bool homogeneous() const { return true; }
		virtual bool nonSpecular() const { return true; }
		virtual bool isEmissive() const { return false; }
		virtual bool hasCosine() const { return true; }
		virtual bool anisotropic() const { return false; }
		virtual vec3f randianceDecay(const Ray &inRay, const float &dist) const { return vec3f(1,1,1); }

	public:
		AnimatedTransform *animatedTransform;
		virtual bool hasMotionBlur() const { return animatedTransform && animatedTransform->actuallyAnimated; }
		virtual void updateTransform(); 
	};
}