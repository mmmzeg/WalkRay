#include "mcRenderer.h"
#include <omp.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace LFJ{
	class PathTracer : public MCRenderer{
		bool useNextEventEstimation;
	public:
		PathTracer(RenderEngine *engine, uint spp, uint maxLen) :
			MCRenderer(engine, spp, maxLen)
		{
			name = "PathTracer";
			useNextEventEstimation = false;
		}

		std::vector<vec3f> renderPixels();
	};
}
