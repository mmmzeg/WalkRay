#pragma once
#include "macros.h"
#include <vector>

namespace LFJ{
	class AbstractObject;
	


	class Ray{
	public:
		enum PhotonType{ INVOL, OUTVOL, NOUSE, HITVOL };
		vec3f origin;
		vec3f direction;


		vec3f radiance;
		float originProb;
		float directionProb;

		AbstractObject *intersectObj;
		float			intersectDist;
		uint			intersectTriangleID;

		AbstractObject *insideObj;
		AbstractObject *contactObj;
		uint			contactTriangleID;

		uint pixelID;
		bool isDeltaDirection;
		PhotonType photonType;

		Ray(); 
		vec3f evalBSDF(const Ray &outRay) const;
		float evalOriginProbability(const Ray &outRay) const;
		float evalDirectionProbability(const Ray &outRay) const;
		vec3f contactNormal() const; 
		vec3f radianceDecay(const float &dist) const; 
		float cosineTerm() const;
	};

	typedef std::vector<Ray> Path;
}