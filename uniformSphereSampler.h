#pragma once
#include "sampler.h"
namespace LFJ{
	class UniformSphereSampler : public Sampler{
	public:
		vec3f sample(Rng &rng, const vec3f &localFix, float *oPdfW = NULL) const{
			const vec2f sample = rng.genVec2f();
			const float term1 = 2.f * PI * sample.x;
			const float term2 = 2.f * std::sqrt(sample.y - sample.y * sample.y);

			const vec3f ret(
				std::cos(term1) * term2,
				1.f - 2.f * sample.y,
				std::sin(term1) * term2
			);

			if(oPdfW)
				*oPdfW = INV_PI * 0.25f;
			return ret;
		}

		float pdf(const vec3f &localFix, const vec3f &localGen) const{ return INV_PI * 0.25f; }

	};
}