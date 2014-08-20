#pragma once
#include "scene.h"
#include "mcRenderer.h"
#include <time.h>
#include <omp.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace LFJ{
	class MCRenderer;
	class Config;
	/*\brief: This is the top class */
	class RenderEngine{
	public:
		/*\brief: random generator */
		Rng *rng;

		/*\brief: a renderer to render the scene */
		/* represents an rendering algorithm     */
		MCRenderer *renderer; 
		
		/*\brief: a scene for renderer to render      */
		/* represents an application of the algorithm */
		Scene scene;

		/*\brief: settings manager */
		Config *config;
	public:
		RenderEngine();
		~RenderEngine();
		void runConfig(const std::string &configFilePath);
		void renderFilm();
		void window();
		void preview();
		void waitCmd();
	};
}
