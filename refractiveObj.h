#include "abstractObj.h"

namespace LFJ{
	class RefractiveObject : public AbstractObject{
		float IOR;
		vec3f decayColor;
	public:
		RefractiveObject(Scene *scene) : AbstractObject(scene)
		{
			IOR = 1.5;
			decayColor = vec3f(1,1,1);
		}
		vec3f evaluateBSDF(const Ray &inRay, const Ray &outRay) const{
			return vec3f(0,0,0);
		}
		vec3f randianceDecay(const Ray &inRay, const float &dist) const { 
			return vec3f(
				std::powf(decayColor.x, dist), 
				std::powf(decayColor.y, dist), 
				std::powf(decayColor.z, dist)
			);
		}
		void setIOR(const float &IOR) { this->IOR = IOR; }
		float getIOR() const { return IOR; }
		void setDecayColor(const vec3f &dc) { decayColor = dc; }
		bool nonSpecular() const { return false; }
		Ray scatter(Ray &inRay) const{
			Ray outRay;
			vec3f position = inRay.origin + inRay.direction * inRay.intersectDist;
			vec3f worldNormal = inRay.intersectObj->getWorldNormal(inRay.intersectTriangleID, position);
			
			if(inRay.intersectObj != this && inRay.insideObj == this)
				return inRay.intersectObj->scatter(inRay);

			outRay.origin = position;
			outRay.direction = inRay.direction;
			vec3f reflDir = -worldNormal.dot(inRay.direction)*worldNormal*2 + inRay.direction;
			reflDir.normalize();
			float theta = acos(inRay.direction.dot(worldNormal));

			AbstractObject *curInsideObj = inRay.insideObj;
			AbstractObject *curOutsideObj = (AbstractObject*)this;
			if(inRay.insideObj == this)
				curOutsideObj = scene->findInsideObject(outRay, (AbstractObject*)this);
			float current_n = curInsideObj ? curInsideObj->getIOR() : 1;
			float next_n = curOutsideObj ? curOutsideObj->getIOR() : 1;
			float sin_phi = current_n / next_n * sin(theta);
			
			outRay.contactObj = (AbstractObject*)this;
			outRay.contactTriangleID = inRay.intersectTriangleID;

			if(sin_phi > 1){
				outRay.direction = reflDir;
				outRay.insideObj = inRay.insideObj;
			}
			else{
				float phi = asin(sin_phi);
				if(theta > PI/2)	phi = PI - phi;
				vec3f axis = worldNormal.cross(inRay.direction);
				axis.normalize();
				outRay.direction = vec3f(RotateMatrix(axis, phi) * vec4f(worldNormal, 0));
				outRay.direction.normalize();

				float cos_theta = abs(cos(theta));
				float cos_phi = abs(cos(phi));
				float esr = pow(abs(current_n*cos_theta-next_n*cos_phi)/(current_n*cos_theta+next_n*cos_phi),2);
				float epr = pow(abs(next_n*cos_theta-current_n*cos_phi)/(next_n*cos_theta+current_n*cos_phi),2);
				float er = (esr+epr)/2;
				
				float p = er;
				if(rng->genFloat() < p){
					outRay.direction = reflDir;
					outRay.radiance *= er / outRay.cosineTerm();
					outRay.directionProb = p;
					outRay.insideObj = inRay.insideObj;
				}
				else{
					outRay.radiance *= (1-er) / outRay.cosineTerm();
					outRay.directionProb = 1-p;
					outRay.insideObj = curOutsideObj;
				}
			}
			outRay.direction.normalize();
			
			outRay.isDeltaDirection = true;

			return outRay;
		}
	};
}