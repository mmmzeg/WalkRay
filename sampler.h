#pragma once
#include "rng.h"
#include "frame.h"
namespace LFJ{
	class Sampler{
	public:
		virtual vec3f sample(Rng &rng, const vec3f &localFix, float *oPdfW = NULL) const{ return vec3f(0,0,0); }
		virtual float pdf(const vec3f &localFix, const vec3f &localGen) const{ return 0; }
	};
}
