#pragma once
#include "macros.h"
#include "quaternion.h"

namespace LFJ{
	class AnimatedTransform
	{
	public:
		AnimatedTransform(Transform transform1, float time1,
			Transform transform2, float time2)
			: startTime(time1), endTime(time2),
			startTransform(transform1), endTransform(transform2)
		{
			Decompose(startTransform, &T[0], &R[0], &S[0]);
			Decompose(endTransform, &T[1], &R[1], &S[1]);
			/*actuallyAnimated = (startTransform != endTransform);*/
			actuallyAnimated = !(startTransform == endTransform);
			std::cout << "actuallyAnimated = " << actuallyAnimated << std::endl;
		}
		
		void Interpolate(float time, Transform *t) const;
		static void Decompose(const Matrix4x4 &m, Vector *T, Quaternion *R, Matrix4x4 *S);
		
		bool actuallyAnimated;
	private:
		// AnimatedTransform Private Data
		float startTime, endTime;
		Transform startTransform, endTransform;
		Vector T[2];
		Quaternion R[2];
		Matrix4x4 S[2];
	};
}

