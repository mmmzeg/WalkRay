#include "mcRenderer.h"
#include <omp.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace LFJ{
	class BidirectionalPathTracer : public MCRenderer{
		float connectingMisWeight(const Path& connectedPath, int connectLightID, float expTerm = 1) const;
		vec4f connectingColorProb(const Path &connectedPath, int connectLightID) const;
		bool  connectEdge(Path &path, int connectLightID);
		Ray   linkVertex(const Path &path, int connectLightID, int startID, bool reverse) const;
		bool  pathCanNotConnect(const Path &path) const;

		void throughputByConnecting(std::vector<omp_lock_t> &pixelLocks, std::vector<vec3f> &colors, 
			const Path &eyePath, const Path &lightPath);
	public:
		BidirectionalPathTracer(RenderEngine *engine, uint spp, uint maxLen) :
			MCRenderer(engine, spp, maxLen)
		{
			name = "BidirectionalPathTracer";
		}
		std::vector<vec3f> renderPixels();
	};
}