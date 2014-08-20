#pragma once
#include "macros.h"
#include <random>

namespace LFJ{
	class Rng{
		std::mt19937_64 mRng;
		std::uniform_int_distribution<int>		mDistInt;
		std::uniform_int_distribution<uint>		mDistUint;
		std::uniform_real_distribution<float>	mDistFloat;
		std::uniform_real_distribution<double>	mDistDouble;
	public:
		Rng(int seed = 1234):
			mRng(seed)
		{}

		int genInt(){
			return mDistInt(mRng);
		}

		uint genUint(){
			return mDistUint(mRng);
		}

		float genFloat(){
			return mDistFloat(mRng);
		}

		double genDouble(){
			return mDistDouble(mRng);
		}

		vec2f genVec2f(){
			float a = genFloat();
			float b = genFloat();
			return vec2f(a,b);
		}

		vec3f genVec3f(){
			float a = genFloat();
			float b = genFloat();
			float c = genFloat();
			return vec3f(a,b,c);
		}

		vec2d genVec2d(){
			double a = genDouble();
			double b = genDouble();
			return vec2d(a,b);
		}

		vec3d genVec3d(){
			double a = genDouble();
			double b = genDouble();
			double c = genDouble();
			return vec3d(a,b,c);
		}
		
	};
}