#include "stdafx.h"
#include "ray.h"
#include "abstractObj.h"
namespace LFJ{
		Ray::Ray(){
			insideObj = contactObj = intersectObj = NULL;
			radiance = vec3f(1,1,1);
			direction = vec3f(0,0,0);
			originProb = directionProb = 1.f;
			pixelID = -1;
			isDeltaDirection = false;
			photonType = OUTVOL;
		}

		vec3f Ray::evalBSDF(const Ray &outRay) const{
			vec3f bsdfValue(0,0,0);
			if(outRay.contactObj)
				bsdfValue = outRay.contactObj->evaluateBSDF(*this, outRay);
			else if(outRay.insideObj)
				bsdfValue = outRay.insideObj->evaluateBSDF(*this, outRay);
			return bsdfValue;
		}		

		float Ray::evalOriginProbability(const Ray &outRay) const{
			float prob = 1;
			if(insideObj)
				prob = insideObj->evalOriginProbability(*this, outRay);
			return prob;
		}

		float Ray::evalDirectionProbability(const Ray &outRay) const{
			float prob = 0;
			if(outRay.contactObj)
				prob = outRay.contactObj->evalDirectionProbability(*this, outRay);
			else if(outRay.insideObj)
				prob = outRay.insideObj->evalDirectionProbability(*this, outRay);
			return prob;
		}

		vec3f Ray::contactNormal() const{
			return contactObj ?
				contactObj->getWorldNormal(contactTriangleID, origin) : 
				vec3f(0,0,0);
		}

		vec3f Ray::radianceDecay(const float &dist) const{
			return insideObj ? 
				insideObj->randianceDecay(*this, dist) : 
				vec3f(1,1,1);
		}

		float Ray::cosineTerm() const{
			float cosine_term = 1;
			if(contactObj && contactObj->hasCosine() && direction.length() > 0.5){
				cosine_term = abs(contactNormal().dot(direction));
			}
			return MAX(cosine_term, EPSILON);
		}
}