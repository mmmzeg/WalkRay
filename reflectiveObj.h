#include "abstractObj.h"
#include <algorithm>

namespace LFJ{
	class ReflectiveObject : public AbstractObject{
		vec3f color;
	public:
		ReflectiveObject(Scene *scene) : AbstractObject(scene)
		{
			color = vec3f(1,1,1);
		}
		void setColor(const vec3f &c) { color = c; }
		vec3f evaluateBSDF(const Ray &inRay, const Ray &outRay) const{
			return vec3f(0,0,0);
		}
		bool nonSpecular() const { return false; }
		Ray scatter(Ray &inRay) const{
			//std::cout << "reflective scattering " << std::endl;
			Ray outRay;
			vec3f position = inRay.origin + inRay.direction * inRay.intersectDist;
			vec3f worldNormal = inRay.intersectObj->getWorldNormal(inRay.intersectTriangleID, position);
			outRay.insideObj = inRay.insideObj;
			
			outRay.origin = position;
			vec3f reflDir = -worldNormal.dot(inRay.direction)*worldNormal*2 + inRay.direction;
			reflDir.normalize();
			outRay.direction = reflDir;
		 
			float p = MAX_VEC_COMP(color);
			if(rng->genFloat() < p){
				outRay.contactObj = (AbstractObject*)this;
				outRay.contactTriangleID = inRay.intersectTriangleID;
				outRay.radiance = color / outRay.cosineTerm();
				outRay.directionProb = p;
				outRay.isDeltaDirection = true;
			}
			else{
				outRay.radiance = vec3f(0,0,0);
				outRay.direction = vec3f(0,0,0);
				outRay.directionProb = 1-p;
			}
			return outRay;
		}
		float evalDirectionProbability(const Ray &inRay, const Ray &outRay) const{
			return MAX_VEC_COMP(color);
		}
	};
}