#include "stdafx.h"
#include "mcRenderer.h"
#include "noself.h"
#include "abstractObj.h"
#include "renderEngine.h"
#include <omp.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
namespace LFJ{
	std::vector<vec3f> MCRenderer::renderPixels(){
		const Camera &camera = getCamera();
		uint width = camera.mResolution.x, height = camera.mResolution.y;
		return std::vector<vec3f>(width * height, vec3f(0,0,0));
	}

	void MCRenderer::samplePath(Path &path, Ray &prevRay, uint depth) const{
		//std::cout << "depth = " << depth << " prevRay " << prevRay.origin << ' '<< prevRay.direction << std::endl;
		path.push_back(prevRay);

		Ray terminateRay;
		terminateRay.origin = prevRay.origin;
		terminateRay.radiance = vec3f(0,0,0);
		terminateRay.direction = vec3f(0,0,0);
		terminateRay.isDeltaDirection = true;
		terminateRay.insideObj = NULL;
		terminateRay.contactObj = NULL;
		terminateRay.intersectObj = NULL;

		Ray nextRay;
		if(prevRay.insideObj)			nextRay = prevRay.insideObj->scatter(prevRay);
		else if(prevRay.intersectObj)	nextRay = prevRay.intersectObj->scatter(prevRay);
		else{
			path.push_back(terminateRay);	return ;
		}

		if(nextRay.direction.length() < 0.5){
			path.push_back(nextRay);		return ;
		}
		if(depth + 1 > maxPathLen){
			path.push_back(terminateRay);	return ;
		}
		NoSelfCondition condition(&engine->scene, nextRay);
		Scene::ObjSourceInfo info;
		float dist = engine->scene.intersect(nextRay, info, &condition);
		if(dist < 0){
			path.push_back(nextRay);
			path.push_back(terminateRay);
			return ;
		}
		else{
			nextRay.intersectObj = engine->scene.objects[info.objID];
			nextRay.intersectTriangleID = info.triID;
			nextRay.intersectDist = dist;
		}
		//std::cout << "nextRay " << nextRay.origin << ' '<< nextRay.direction<<std::endl;
		samplePath(path, nextRay, depth + 1);
	}

	bool MCRenderer::visibilityTest(const Ray &inRay, const Ray &outRay) const{
		NoSelfCondition condition(&engine->scene, inRay);
		Scene::ObjSourceInfo info;
		float dist = engine->scene.intersect(inRay, info, &condition);
		return dist < 0 || 
			(dist - (inRay.origin-outRay.origin).length() >= -EPSILON);
	}

	Camera& MCRenderer::getCamera(){
		return engine->scene.getCamera();
	}

	void MCRenderer::prepareForLightSampling(){
		engine->scene.prepareForLightSampling();
	}

	Ray MCRenderer::genLightSample() const{
		return engine->scene.genLightSample();
	}

	void MCRenderer::showCurrentResult(const std::string &savePath,  std::vector<vec3f> &pixelColors) {
		int width = engine->scene.mCamera.mResolution.x, height = engine->scene.mCamera.mResolution.y;
		IplImage *img = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
		for(int x = 0; x < width; x++){
			for(int y = 0; y < height; y++){
				vec3f rgb = pixelColors[y*width + x];
				vec3f &bgr = ((vec3f*)img->imageData)[y*width + x];
				bgr = vec3f(rgb.z, rgb.y, rgb.x);
			}
		}

		saveImagePFM(savePath, img);
		/*cvShowImage("WalkRay", img);*/
		
		/*int key = cvWaitKey(5);
		if(key == 's' || key == 'S'){
			engine->scene.mCamera.mFilm.setBuffer(pixelColors);
			std::string filename = engine->renderer->name + engine->scene.name + ".pfm";
			engine->scene.mCamera.mFilm.savePFM(filename);
		}
*/
		cvReleaseImage(&img);
	}

	void MCRenderer::saveImagePFM(const std::string& fileName, const IplImage* image)
	{
		FILE* file;
		fopen_s(&file, fileName.c_str(), "wb");

		fprintf(file, "PF\n%d %d\n-1.000000\n", image->width, image->height);

		const float* data = (float*)image->imageData;
		for(int j=image->height-1; j>=0; j--)
		{
			for(unsigned i=0; i<image->width; i++)
			{
				fwrite(&data[3*(j*image->width+i)+2], sizeof(float), 1, file);
				fwrite(&data[3*(j*image->width+i)+1], sizeof(float), 1, file);
				fwrite(&data[3*(j*image->width+i)], sizeof(float), 1, file);
			}
		}
		fclose(file);
	}

}