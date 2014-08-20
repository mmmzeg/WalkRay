#include "stdafx.h"
#include "camera.h"
#include "scene.h"

namespace LFJ{
	Ray Camera::generateRay(uint pixelID) const{
		vec3f front = mFocus - mPosition;
		front.normalize();
		vec3f right = front.cross(mUp);
		right.normalize();
		vec3f top = right.cross(front);
		Ray ray;
		uint width = mResolution.x, height = mResolution.y;
		uint x = pixelID % width;
		uint y = pixelID / width;
		float u = mRng->genFloat();
		float v = mRng->genFloat();
		// add for PPM Research
		// u = v = 0;
		ray.direction = front * mImagePlaneDist +
			right * (x + u - (width/2.f)) +
			top * ((height/2.f) - y - v);
		ray.direction.normalize();
		ray.origin = mPosition;
		ray.radiance = vec3f(1,1,1);
		ray.directionProb = 1.0;   //  1.0 / getPixelSolidAngle(pixelID); 
		ray.pixelID = pixelID;

		// Modify ray for depth of field
		if(mLensRadius > 0.f){
			// sample point on lens
			float lensU, lensV;
			vec2f sample2D = mRng->genVec2f();
			concentricSampleDisk(sample2D.x, sample2D.y, &lensU, &lensV);
			lensU *= mLensRadius;
			lensV *= mLensRadius;

			// compute point on plane of focus
			float cosTheta = abs(front.dot(ray.direction));
			float ft = mFocalDist / cosTheta;
			vec3f pFocus = ray.origin + ray.direction * ft;

			// update ray for effect of lens
			ray.origin = mPosition + lensU * right + lensV * top;
			ray.direction = pFocus - ray.origin;	
			ray.direction.normalize();
		}



		Scene::ObjSourceInfo info;
		ray.intersectDist = scene->intersect(ray, info);
		ray.intersectObj = ray.intersectDist>=0 ? scene->objects[info.objID] : NULL;
		ray.intersectTriangleID = info.triID;
		ray.contactObj = (AbstractObject*)this;
		ray.insideObj = scene->findInsideObject(ray, NULL);
		ray.isDeltaDirection = false;

		return ray;
	}

	void Camera::setDOFParameters(float lensRadius, float focalDist){
		mLensRadius = lensRadius;
		mFocalDist = focalDist;
	}

	std::vector<vec3f> Camera::generateRays() const{
		uint width = mResolution.x, height = mResolution.y;
		std::vector<vec3f> rays(width * height);
		vec3f front = mFocus - mPosition;
		front.normalize();
		vec3f right = front.cross(mUp);
		right.normalize();
		vec3f top = right.cross(front);

		std::cout << "generateRays " << front << right << top << std::endl;

		for(uint y = 0; y < height; y++){
			for(uint x = 0; x < width; x++){
				rays[y*width + x] = front * mImagePlaneDist + 
					right * (x + 0.5 - (width/2.0)) +
					top * ((height/2.0) - 0.5 - y);
				rays[y*width + x].normalize();
			}
		}

		return rays;
	}

	vec3f Camera::fixVignetting(const vec3f &color, uint pixelID) const{
		/*uint width = mResolution.x, height = mResolution.y;
		vec3f front = mFocus - mPosition;
		front.normalize();
		vec3f right = front.cross(mUp);
		right.normalize();
		vec3f top = right.cross(front);
		uint x = pixelID % width, y = pixelID / width;
		vec3f dir =	front * mImagePlaneDist + 
		right * (x + 0.5 - (width/2.0)) +
		top * ((height/2.0) - 0.5 - y);
		dir.normalize();
		if(dir.dot(front) == 0)
		return 0.0;
		vec3f result = color / powf(dir.dot(front), 4) * (mImagePlaneDist * mImagePlaneDist);

		return isLegalColor(result) ? result : vec3f(0,0,0);*/
		// now only fix cosine term caused by eyeRay[0]
		uint width = mResolution.x, height = mResolution.y;
		vec3f front = mFocus - mPosition;
		front.normalize();
		vec3f right = front.cross(mUp);
		right.normalize();
		vec3f top = right.cross(front);
		uint x = pixelID % width, y = pixelID / width;
		vec3f dir =	front * mImagePlaneDist + 
			right * (x + 0.5 - (width/2.0)) +
			top * ((height/2.0) - 0.5 - y);
		dir.normalize();
		if(dir.dot(front) == 0)
			return 0.0;
		vec3f result = color / fabs(dir.dot(front));
		return isLegalColor(result) ? result : vec3f(0,0,0);
	}

	float Camera::getPixelSolidAngle(uint pixelID) const{
		uint width = mResolution.x, height = mResolution.y;
		uint x = pixelID % width, y = pixelID / width;
		float xc = x + 0.5 -(width/2.0), yc = (height/2.0) - 0.5 - y;
		float pixelDist = sqrt(xc*xc + yc*yc + mImagePlaneDist*mImagePlaneDist);
		return mImagePlaneDist / powf(pixelDist,3);
	}

	float Camera::computeSolidAngle(const vec3f &p, const vec3f &o) const{
		vec3f shift = o - mPosition;
		vec3f p2 = p - shift;
		vec3f dir = p - o;					dir.normalize();

		vec3f front = mFocus - mPosition;	front.normalize();
		float cosTheta = dir.dot(front);
		float pixelDist = mImagePlaneDist / cosTheta;
		return mImagePlaneDist / powf(pixelDist,3);
	}

	float Camera::evalDirectionProbability(const Ray &inRay, const Ray &outRay) const{
		return 1.0 / getPixelSolidAngle(outRay.pixelID);
	}

	vec2f Camera::convToRaster(const vec3f &p) const{
		vec2f pixel;
		vec3f v = p - mPosition;
		uint width = mResolution.x, height = mResolution.y;
		vec3f front = mFocus - mPosition;
		front.normalize();
		vec3f right = front.cross(mUp);
		right.normalize();
		vec3f top = right.cross(front);
		float vFrontDist = v.dot(front);
		float scale = mImagePlaneDist / vFrontDist;
		pixel.x = width/2.0 + v.dot(right) * scale;
		pixel.y = height/2.0 - v.dot(top) * scale;
		return pixel;
	}

	vec2f Camera::convToRaster2(const vec3f &p, const vec3f &o) const{
		vec3f shift = o - mPosition;
		vec3f dir = p - o;						dir.normalize();
		vec3f front = mFocus - mPosition;		front.normalize();
		float cosTheta = abs(front.dot(dir));
		float ft = mFocalDist / cosTheta;
		vec3f pFocus = o + dir * ft;
		return convToRaster(pFocus);
	}

	bool Camera::checkPixelBound(vec2f coord) const{
		int width = mResolution.x, height = mResolution.y;
		int pX = coord.x, pY = coord.y;
		if(pX >= 0 && pY >= 0 && pX < width && pY < height)
			return true;
		return false;
	}

	vec3f Camera::getWorldNormal(uint fi, const vec3f &pos, bool flat) const{
		vec3f front = mFocus - pos;
		front.normalize();
		return front;
	}

	vec3f Camera::sampleLensPoint() const{
		uint width = mResolution.x, height = mResolution.y;
		vec3f front = mFocus - mPosition;
		front.normalize();
		vec3f right = front.cross(mUp);
		right.normalize();
		vec3f top = right.cross(front);

		// sample point on lens
		float lensU, lensV;
		vec2f sample2D = mRng->genVec2f();
		concentricSampleDisk(sample2D.x, sample2D.y, &lensU, &lensV);
		lensU *= mLensRadius;
		lensV *= mLensRadius;

		return mPosition + lensU * right + lensV * top;
	}


	void Camera::concentricSampleDisk(float u1, float u2, float *dx, float *dy) const{
		float r, theta;
		// Map uniform random numbers to $[-1,1]^2$
		float sx = 2 * u1 - 1;
		float sy = 2 * u2 - 1;

		// Map square to $(r,\theta)$

		// Handle degeneracy at the origin
		if (sx == 0.0 && sy == 0.0) {
			*dx = 0.0;
			*dy = 0.0;
			return;
		}
		if (sx >= -sy) {
			if (sx > sy) {
				// Handle first region of disk
				r = sx;
				if (sy > 0.0) theta = sy/r;
				else          theta = 8.0f + sy/r;
			}
			else {
				// Handle second region of disk
				r = sy;
				theta = 2.0f - sx/r;
			}
		}
		else {
			if (sx <= sy) {
				// Handle third region of disk
				r = -sx;
				theta = 4.0f - sy/r;
			}
			else {
				// Handle fourth region of disk
				r = -sy;
				theta = 6.0f + sx/r;
			}
		}
		theta *= PI / 4.f;
		*dx = r * cosf(theta);
		*dy = r * sinf(theta);
	}
}