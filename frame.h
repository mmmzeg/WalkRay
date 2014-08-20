#pragma once
#include "macros.h"

namespace LFJ{
	class Frame{
	public:
		Frame(){
			mX = vec3f(1,0,0);
			mY = vec3f(0,1,0);
			mZ = vec3f(0,0,1);
		}
		
		Frame(
			const vec3f &x,
			const vec3f &y,
			const vec3f &z
		) :
			mX(x),
			mY(y),
			mZ(z)
		{}

		vec3f toWorldFrame(const vec3f &w) const{
			return mX * w.x + mY * w.y + mZ * w.z;
		}

		vec3f toLocalFrame(const vec3f &l) const{
			return vec3f(l.dot(mX), l.dot(mY), l.dot(mZ));
		}

		void setFromN(const vec3f &w){
			/*mY = w;
			mX = ((fabs(w.x)>.1?vec3f(0,1,0):vec3f(1,0,0)).cross(w));
			mX.normalize();
			mZ = mX.cross(mY);*/

			mY = w;
			if(fabs(w.x) > EPSILON){
				mX = vec3f(0,1,0).cross(w);
				mX.normalize();
				mZ = mX.cross(mY);
			}
			else{
				mZ = vec3f(1,0,0).cross(w);
				mZ.normalize();
				mX = mY.cross(mZ);
			}
			
		}

		void setFromDisk(const vec3f &N, const vec3f &pos, const vec3f &center){
			mY = N;
			vec3f dir = pos - center;	
			
			if(fabs(dir.length()) < EPSILON){	setFromN(N);	return ; } 
			
			dir.normalize();
			mZ = dir;
			mX = mY.cross(mZ);
			mX.normalize();

			//std::cout << "disk LF: " << mX << " " << mY << " " << mZ << std::endl;
		}

	public: 
		vec3f mX, mY, mZ;
	}; 


}
