#pragma once
#include "sampler.h"
namespace LFJ{
	class CosHemiSphereSampler : public Sampler{
	public:
		vec3f sample(Rng &rng, const vec3f &localFix, float *oPdfW = NULL) const{
			const vec2f sample = rng.genVec2f();
			const float term1 = 2.f * PI * sample.x;
			const float term2 = std::sqrt(1.f - sample.y);

			const vec3f ret(
				std::sin(term1) * term2,
				std::sqrt(sample.y),
				std::cos(term1) * term2
			);

			return ret;
		}

		float pdf(const vec3f &localFix, const vec3f &localGen) const{ 
			// assert not calling this
			return localGen.y * INV_PI;
		}

	};
}