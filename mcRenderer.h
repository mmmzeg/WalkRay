#pragma once
#include "macros.h"
#include "ray.h"
#include "renderEngine.h"
#include <vector>


#include <omp.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
namespace LFJ{
	class RenderEngine;

	class MCRenderer{
	protected:
		uint maxPathLen; // maximum length of one path
		uint spp; // samples per pixel 
		RenderEngine *engine; 
	public:
		std::string name;

	public:
		MCRenderer(RenderEngine *engine, uint spp, uint maxLen) :
			engine(engine), spp(spp), maxPathLen(maxLen)
		{}

		virtual std::vector<vec3f> renderPixels();
		void samplePath(Path &path, Ray &prevRay, uint depth = 0) const;
		bool visibilityTest(const Ray &inRay, const Ray &outRay) const;
		Camera& getCamera();
		void prepareForLightSampling();
		Ray genLightSample() const;
		void showCurrentResult(const std::string &p, std::vector<vec3f> &pixelColors) ;
		void handleEvent() const;
		void saveImagePFM(const std::string& fileName, const IplImage* image);
	};
}