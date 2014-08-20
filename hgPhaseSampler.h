#pragma once
#include "sampler.h"

namespace LFJ{
	class HGPhaseSampler : public Sampler{
		float g;
	public:
		HGPhaseSampler(float g) : g(g) {}
		vec3f sample(Rng &rng, const vec3f &localFix, float *oPdfW = NULL) const{ 
			vec3f rayDir = localFix;
			rayDir.normalize();

			float e1 = rng.genFloat(), e2 = rng.genFloat();
			float cosTheta;
			if(abs(g) < EPSILON){
				cosTheta = 1 - 2*e1;
			}
			else{
				float sqrTerm = (1 - g*g) / (1 - g + 2*g*e1);
				cosTheta = (1 + g*g - sqrTerm * sqrTerm) / (2*g);
			}
			float sinTheta = sqrt(1 - cosTheta * cosTheta), sinPhi, cosPhi;
			sinPhi = sin(2.0 * PI * e2);
			cosPhi = cos(2.0 * PI * e2);

			vec3f localGenDir = vec3f(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta), u, v;
			Frame newLf = generateOrthoBasis(rayDir);
			u = newLf.mX;	v = newLf.mY;

			localGenDir = u*localGenDir.x + v*localGenDir.y + rayDir*localGenDir.z;
			
			if(oPdfW)
				*oPdfW = pdf(rayDir, localGenDir);

			return localGenDir;
		}

		float pdf(const vec3f &localFix, const vec3f &localGen) const{ 
			float cosTheta = localFix.dot(localGen);
			float temp = 1 + g*g - 2*g*cosTheta;
		
			return 1/(4*PI) * (1-g*g) / (temp*sqrt(temp));
		}

		Frame generateOrthoBasis(vec3f w) const{
			vec3f coVec = w, u, v;
			if (fabs(w.x) <= fabs(w.y))
				if (fabs(w.x) <= fabs(w.z)) coVec = vec3f(0,-w.z,w.y);
				else coVec = vec3f(-w.y,w.x,0);
			else if (fabs(w.y) <= fabs(w.z)) coVec = vec3f(-w.z,0,w.x);
			else coVec = vec3f(-w.y,w.x,0);
			coVec.normalize();
			u = w.cross(coVec);
			v = w.cross(u);
			Frame lf;
			lf.mX = u;
			lf.mY = v;
			return lf;
		}
	};
}
