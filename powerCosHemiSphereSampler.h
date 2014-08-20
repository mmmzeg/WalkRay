#pragma once
#include "sampler.h"
namespace LFJ{
	class PowerCosHemiSphereSampler : public Sampler{
		float power;
	public:
		PowerCosHemiSphereSampler(float p) : power(p) {}
		vec3f sample(Rng &rng, const vec3f &localFix, float *oPdfW = NULL) const{
			const vec2f sample = rng.genVec2f();
			const float term1 = 2.f * PI * sample.x;
			const float term2 = std::pow(sample.y, 1.0 / (power + 1.0));
			const float term3 = std::sqrt(1.0 - term2 * term2);

			const vec3f ret(
				std::sin(term1) * term3,
				term2,
				std::cos(term1) * term3
			);

			return ret;
		}

		float pdf(const vec3f &localFix, const vec3f &localGen) const{ 
			// assert not calling this function
			const float cosTheta = localGen.y;
			return (power + 1.0) * std::pow(cosTheta, power) * (INV_PI * 0.5);
		}

	};
}