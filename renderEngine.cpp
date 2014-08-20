#include "stdafx.h"
#include "renderEngine.h"
#include "mcRenderer.h"
#include "abstractObj.h"
#include "config.h"

static int loadTime = 0;

namespace LFJ{
	RenderEngine::RenderEngine(){
		clock_t seed = time(NULL);//clock();
		rng = new Rng(seed);
		scene.setRng(rng);
		renderer = NULL;
		config = NULL;
	}

	void RenderEngine::runConfig(const std::string &configFilePath){
		scene.clear();
		if(renderer)		{ delete renderer;  renderer = NULL; }
		if(config)			{ delete config;	config   = NULL; }
		config = new Config(this);
		config->loadTime = ++loadTime;
		config->load(configFilePath);
	}

	void RenderEngine::renderFilm(){
		scene.buildObjKDTrees();
		renderer->renderPixels();
	}

	void RenderEngine::window(){
		cvNamedWindow("WalkRay");
	}

	void RenderEngine::preview(){
		uint width = scene.mCamera.mResolution.x, height = scene.mCamera.mResolution.y;
		IplImage *img = cvCreateImage(cvSize(scene.mCamera.mResolution.x, scene.mCamera.mResolution.y), IPL_DEPTH_32F, 3);
		std::vector<vec3f> eyeRays = scene.mCamera.generateRays();
#pragma omp parallel for
		for(int x = 0; x < width; x++){

			class SmoothNormal : public KDTree::Condition{
			public:
				Scene *scene;
				bool legal(const KDTree::Ray &ray, const KDTree::Triangle& tri, const float dist) const{
					AbstractObject *intersectObject = scene->objects[((Scene::ObjSourceInfo*)tri.sourceInformation)->objID];
					uint fi = ((Scene::ObjSourceInfo*)tri.sourceInformation)->triID;
					bool in = ray.direction.dot(intersectObject->getWorldNormal(fi, ray.origin + ray.direction*dist))<0;
					return in;
				}
			} condition;

			for(int y = 0; y < height; y++){
				condition.scene = &scene;
				Ray eyeRay;
				eyeRay.direction = eyeRays[y*width + x];
				eyeRay.origin = scene.mCamera.mPosition;
				Scene::ObjSourceInfo info;
				float dist = scene.intersect(eyeRay, info/*, &condition*/);
				vec3f normal = dist >= 0 ? 
					scene.objects[info.objID]->getWorldNormal(info.triID, eyeRay.origin + eyeRay.direction * dist) : 
					vec3f(0,0,0);
				((vec3f*)img->imageData)[y*width + x] = vec3f(1,1,1) * abs(eyeRay.direction.dot(normal));
			}
		}

		cvShowImage("WalkRay", img);
		cvReleaseImage(&img);
	}

	void RenderEngine::waitCmd(){
		int key = cvWaitKey(10);
		while(key != 'q'){
			switch(key){
			case 'p':
			case 'P':
				runConfig("data/Config.xml");
				preview();
				break;
			case 'r':
			case 'R':
				cvDestroyAllWindows();
				renderFilm();
				break;
			default:
				break;
			}
			key = cvWaitKey(10);
		}
	}

	RenderEngine::~RenderEngine(){
		if(rng)				delete rng;
		if(renderer)		delete renderer;
		if(config)			delete config;
		rng = NULL;	
		renderer = NULL;
		config = NULL;
		cvDestroyAllWindows();
	}

}