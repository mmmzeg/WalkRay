#pragma once
#include "sampler.h"
#include "ray.h"

namespace LFJ{
	class AbstractObject; 

	class SurfaceSampler : public Sampler{
	public:
		Rng *rng;
		float boundArea;
		std::vector<float> areaThreshold;
		std::vector<AbstractObject*> targetObjects;
	public:
		SurfaceSampler(Rng *rng) : rng(rng) {
			boundArea = 0;
		}
		void prepareForSampling();
		Ray genSample() const;
	};
}