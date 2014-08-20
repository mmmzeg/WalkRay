#include "stdafx.h"
#include "scene.h"
#include "homogeneousVolume.h"

namespace LFJ{

	Ray HomogeneousVolume::scatter(Ray &inRay) const{
		Ray outRay;
		outRay.isDeltaDirection = false;
		float sampDist = sampleDist();

		bool go_in_vol  = inRay.intersectObj == this && inRay.insideObj != this;
		bool be_in_vol  = inRay.insideObj == this;
		bool out_of_vol = sampDist >= inRay.intersectDist; // hit a surface

		// CASE1: Go in volume.
		if(go_in_vol){
			vec3f position = inRay.origin + inRay.direction*inRay.intersectDist;
			vec3f normal = inRay.intersectObj->getWorldNormal(inRay.intersectTriangleID, position);

			outRay.origin = position;
			outRay.direction = inRay.direction;

			vec3f reflDir = -normal.dot(inRay.direction)*normal*2 + inRay.direction;
			reflDir.normalize();
			float theta = acos(inRay.direction.dot(normal));

			AbstractObject* currentInsideObject = inRay.insideObj;
			AbstractObject* outSideObject = (AbstractObject*)this;

			float current_n = currentInsideObject ? currentInsideObject->getIOR() : 1;
			float next_n = outSideObject ? outSideObject->getIOR() : 1;
			float sin_phi = current_n / next_n * sin(theta);

			outRay.intersectObj = NULL;
			outRay.radiance = vec3f(1, 1, 1);
			outRay.directionProb = 1;
			outRay.contactObj = (AbstractObject*)this;
			outRay.contactTriangleID = inRay.intersectTriangleID;

			if(sin_phi > 1){
				outRay.direction = reflDir;
				outRay.insideObj = inRay.insideObj;
				outRay.directionProb = 1;
				outRay.isDeltaDirection = true;
				outRay.photonType = Ray::NOUSE;
			}
			else{
				float phi = asin(sin_phi);
				if(theta > PI/2)	phi = PI - phi;
				vec3f axis = normal.cross(inRay.direction);
				axis.normalize();
				outRay.direction = vec3f(RotateMatrix(axis, phi) * vec4f(normal, 0));
				outRay.direction.normalize();

				float cos_theta = abs(cos(theta));
				float cos_phi = abs(cos(phi));
				float esr = powf(abs(current_n*cos_theta-next_n*cos_phi)/(current_n*cos_theta+next_n*cos_phi),2);
				float epr = powf(abs(next_n*cos_theta-current_n*cos_phi)/(next_n*cos_theta+current_n*cos_phi),2);
				float er = (esr+epr)/2;
				float p = er;

				if(rng->genFloat() < p)
				{
					outRay.direction = reflDir;
					outRay.radiance *= er / outRay.cosineTerm();
					outRay.directionProb = p;
					outRay.insideObj = inRay.insideObj;
					outRay.isDeltaDirection = true;
					outRay.photonType = Ray::NOUSE;
				}
				else
				{
					outRay.radiance *= (1-er) / outRay.cosineTerm();
					outRay.directionProb = 1-p;
					outRay.contactObj = outRay.insideObj = (AbstractObject*)this;
					outRay.isDeltaDirection = true;
					outRay.photonType = Ray::HITVOL;
				}
				outRay.direction.normalize();
			}
			return outRay;
		}


		// CASE2: Be in volume.
		if(be_in_vol && !out_of_vol){
			outRay.origin = inRay.origin + inRay.direction * sampDist;
			outRay.radiance = bsdf->sampleBSDF(inRay.direction, outRay.direction, vec3f(), *rng, &outRay.directionProb);
			outRay.insideObj = (AbstractObject*)this;
			outRay.contactTriangleID = inRay.intersectTriangleID;
			float albedo = getAlbedo();
			float rander = rng->genFloat();	
			//outRay.originSampleType = Ray::RANDOM;
			if(rander < albedo){
				outRay.contactObj = NULL;
				outRay.directionProb *= albedo;
				outRay.originProb = pMedium(sampDist);
				outRay.isDeltaDirection = false;
				outRay.radiance *= scatteringCoeff;
				outRay.photonType = Ray::INVOL;
			}
			else{
				// terminate
				outRay.direction = vec3f(0, 0, 0); 
				outRay.radiance = vec3f(0, 0, 0);  
				outRay.directionProb = 1; 
				outRay.originProb = pMedium(sampDist);
				outRay.insideObj = NULL;
				outRay.contactObj = NULL;
				outRay.isDeltaDirection = false;
				outRay.photonType = Ray::INVOL;
			}
			return outRay;
		}


		// CASE3: Go out of volume.
		if(be_in_vol && out_of_vol){
			outRay = inRay;
			outRay.direction = inRay.direction;
			outRay.origin = inRay.origin + inRay.intersectDist * inRay.direction;
			outRay.contactObj = inRay.intersectObj;
			outRay.contactTriangleID = inRay.intersectTriangleID;
			outRay.insideObj = (AbstractObject*)this;
			outRay.directionProb = 1; 
			outRay.radiance = vec3f(1,1,1);
			bool going_out = (inRay.intersectObj == this);

			if(going_out){
				vec3f normal = inRay.intersectObj->getWorldNormal(inRay.intersectTriangleID, outRay.origin);
				vec3f reflDir = -normal.dot(inRay.direction)*normal*2 + inRay.direction;
				reflDir.normalize();
				float theta = acos(inRay.direction.dot(normal));
				
				AbstractObject* currentInsideObject = (AbstractObject*)this;
				AbstractObject* outSideObject = scene->findInsideObject(outRay, (AbstractObject*)this);

				float current_n = currentInsideObject ? currentInsideObject->getIOR() : 1;
				float next_n = outSideObject ? outSideObject->getIOR() : 1;
				float sin_phi = current_n / next_n * sin(theta);

				outRay.intersectObj = NULL;
				if(sin_phi > 1){
					outRay.direction = reflDir;
					outRay.insideObj = inRay.insideObj;
					outRay.contactObj = (AbstractObject*)this;
					outRay.originProb = PSurface(inRay.intersectDist);
					outRay.photonType = Ray::NOUSE;
					outRay.isDeltaDirection = true;
				}
				else{
					float phi = asin(sin_phi);
					if(theta > PI/2)	phi = PI - phi;
					vec3f axis = normal.cross(inRay.direction);
					axis.normalize();
					outRay.direction = vec3f(RotateMatrix(axis, phi) * vec4f(normal, 0));
					outRay.direction.normalize();

					float cos_theta = abs(cos(theta));
					float cos_phi = abs(cos(phi));
					float esr = powf(abs(current_n*cos_theta-next_n*cos_phi)/(current_n*cos_theta+next_n*cos_phi),2);
					float epr = powf(abs(next_n*cos_theta-current_n*cos_phi)/(next_n*cos_theta+current_n*cos_phi),2);
					float er = (esr+epr)/2;
					float p = er;

					if(rng->genFloat() < p)
					{
						outRay.direction = reflDir;
						outRay.radiance *= er / outRay.cosineTerm();
						outRay.directionProb = p;
						outRay.originProb =  PSurface(inRay.intersectDist);
						outRay.insideObj = inRay.insideObj;
						outRay.isDeltaDirection = true;
						outRay.photonType = Ray::NOUSE;
					}
					else
					{
						outRay.radiance *= (1-er) / outRay.cosineTerm();
						outRay.directionProb = (1-p);
						outRay.originProb = PSurface(inRay.intersectDist);
						outRay.insideObj = outSideObject;
						outRay.isDeltaDirection= true;
						outRay.photonType = Ray::NOUSE;
					}
					outRay.direction.normalize();
				}	
			}
			else{
				outRay.contactObj = NULL;
				outRay.intersectDist = 0;
				outRay = inRay.intersectObj->scatter(outRay);
				outRay.originProb *= PSurface(inRay.intersectDist);
				outRay.photonType = inRay.intersectObj->isVolume() ? Ray::NOUSE : Ray::OUTVOL;
			}
			return outRay;
		}
		return outRay;
	}

	vec3f HomogeneousVolume::evaluateBSDF(const Ray &inRay, const Ray &outRay) const{
		if(!outRay.contactObj)
			return scatteringCoeff * bsdf->evalBSDF(inRay.direction, outRay.direction, vec3f(0,0,0));
		if(outRay.contactObj && outRay.contactObj != this)
			return outRay.contactObj->evaluateBSDF(inRay, outRay);
		return vec3f(0,0,0);
	}

	float HomogeneousVolume::evalOriginProbability(const Ray &inRay, const Ray &outRay) const{
		float dist = MAX((inRay.origin-outRay.origin).length(), EPSILON);

		return outRay.contactObj ?
			PSurface(dist) : pMedium(dist);
	}

	float HomogeneousVolume::evalDirectionProbability(const Ray &inRay, const Ray &outRay) const{
		if(outRay.isDeltaDirection)		return 0;
		if(outRay.contactObj)
			return outRay.contactObj->evalDirectionProbability(inRay, outRay);
		float continueAlbedo = getAlbedo();
		float oPdfW = bsdf->evalPdfW(inRay.direction, outRay.direction, vec3f());
		return continueAlbedo * oPdfW;
	}

	vec3f HomogeneousVolume::randianceDecay(const Ray &inRay, const float &dist) const{
		return tau(dist);
	}
}
