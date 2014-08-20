#include "stdafx.h"
#include "motionBlur.h"
#include "util.h"

namespace LFJ{
	void AnimatedTransform::Interpolate(float time, Transform *t) const {
		// Handle boundary conditions for matrix interpolation

		if(!actuallyAnimated) {
			std::cout << "Motion Blur: Not actually animated!" << std::endl;
			*t = startTransform;
			return;
		}

		if (time <= startTime) {
			std::cout << "Motion Blur: t0 <= startTime " << std::endl;
			*t = startTransform;
			return;
		}

		if (time >= endTime) {
			std::cout << "Motion Blur: t1 >= endTime" << std::endl;
			*t = endTransform;
			return;
		}
		float dt = (time - startTime) / (endTime - startTime);
		// Interpolate translation at _dt_
		Vector trans = (1.f - dt) * T[0] + dt * T[1];

		// Interpolate rotation at _dt_
		Quaternion rotate = Slerp(dt, R[0], R[1]);

		// Interpolate scale at _dt_
		Matrix4x4 scale;
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 3; ++j)
				scale(i,j) = Lerp(dt, S[0](i,j), S[1](i,j));

		// Compute interpolated matrix as product of interpolated components
		Matrix4x4 I;	
		I.make_identity();
		I.set_translate(trans);
		*t = I * rotate.ToTransform() * Transform(scale);

	}

	// AnimatedTransform Method Definitions
	void AnimatedTransform::Decompose(const Matrix4x4 &m, Vector *T,
		Quaternion *Rquat, Matrix4x4 *S) {
			// Extract translation _T_ from transformation matrix
			T->x = m(0,3);
			T->y = m(1,3);
			T->z = m(2,3);

			// Compute new transformation matrix _M_ without translation
			Matrix4x4 M = m;
			for (int i = 0; i < 3; ++i)
				M(i,3) = M(3,i) = 0.f;
			M(3,3) = 1.f;

			// Extract rotation _R_ from transformation matrix
			float norm;
			int count = 0;
			Matrix4x4 R = M;
			do {
				// Compute next matrix _Rnext_ in series
				Matrix4x4 Rnext;
				Matrix4x4 Rit = inverse(transpose(R));

				for (int i = 0; i < 4; ++i)
					for (int j = 0; j < 4; ++j)
						Rnext(i,j) = 0.5f * (R(i,j) + Rit(i,j));

				// Compute norm of difference between _R_ and _Rnext_
				norm = 0.f;
				for (int i = 0; i < 3; ++i) {
					float n = fabsf(R(i,0) - Rnext(i,0)) +
						fabsf(R(i,1) - Rnext(i,1)) +
						fabsf(R(i,2) - Rnext(i,2));
					norm = std::max(norm, n);
				}
				R = Rnext;
			} while (++count < 100 && norm > .0001f);

			// XXX TODO FIXME deal with flip...
			*Rquat = Quaternion(R);

			// Compute scale _S_ using rotation and original matrix
			*S = (inverse(R) * M);
	}
}