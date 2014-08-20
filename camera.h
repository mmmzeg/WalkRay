#pragma once
#include "film.h"
#include "ray.h"
#include "rng.h"
#include "abstractObj.h"

namespace LFJ{
	class Scene; 
	
	class Camera : public AbstractObject{
	public:
	    Rng   *mRng;
		Film   mFilm;
		vec3f  mPosition;
		vec3f  mFocus;
		vec3f  mUp;
		vec2f  mResolution;
		float  mImagePlaneDist;

	private: // for depth of field
		float  mLensRadius; 
		float  mFocalDist;  

	public:
		Camera(Scene *scene) : AbstractObject(scene), 
			mLensRadius(0.f), mFocalDist(std::numeric_limits<float>::max()){}

		void setup(
			const vec3f &pos,
			const vec3f &foc,
			const vec3f &top,
			const vec2f &res,
			const float dist,
			Rng			*rng)
		{
			mPosition   = pos;
			mFocus      = foc;
			mUp         = top;
			mResolution = res;
			mRng		= rng;
			mFilm.setup(mResolution);
			mImagePlaneDist = dist;
			std::cout << "pos " << pos << std::endl
				<< "foc " << foc << std::endl
				<< "up " << top << std::endl
				<< "res " << res << std::endl
				<< "dist " << dist << std::endl;
		}

		void setDOFParameters(float lensRadius, float focalDist);
		vec3f sampleLensPoint() const;
		void concentricSampleDisk(float u1, float u2, float *dx, float *dy) const;
		Ray generateRay(uint pixelID) const;
		std::vector<vec3f> generateRays() const;
		float getPixelSolidAngle(uint pixelID) const;
		float computeSolidAngle(const vec3f &p, const vec3f &o) const;
		vec3f fixVignetting(const vec3f &color, uint pixelID) const;
		vec3f getWorldNormal(uint fi, const vec3f &position, bool flat = false) const;
		vec2f convToRaster(const vec3f &p) const;
		vec2f convToRaster2(const vec3f &p, const vec3f &o) const;
		bool checkPixelBound(vec2f coord) const;
		bool nonSpecular() const { return false; }
		bool useDOF() const { return mLensRadius > 0.f; }
		float evalDirectionProbability(const Ray &inRay, const Ray &outRay) const;
	};
}