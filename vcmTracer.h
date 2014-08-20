#include "mcRenderer.h"
#include "hashgrid.hxx"
#include <omp.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace LFJ{
	class VCMTracer : public MCRenderer{
		float alpha;
		float baseRadius;
		float radius; // radius per iteration

		HashGrid hashGrid;
		omp_lock_t hashGridLock;
		typedef struct HashGridVertex{
			Path  *pathThePointIn;
			uint  indexInThePath;
			vec3f position;
			vec3f GetPosition() const{ return position; }
		} LightPoint;

	private:
		typedef enum {VC, VM} VCMType;
		float vcmMisWeight(const Path& connectedPath, int connectLightID, VCMType vcmType, float expTerm = 1) const;
		vec4f connectingColorProb(const Path &connectedPath, int connectLightID, VCMType vcmType) const;
		bool  connectEdge(Path &path, int connectLightID, VCMType vcmType);
		Ray   linkVertex(const Path &path, int connectLightID, int startID, bool reverse) const;
		bool  pathCanNotConnect(const Path &path) const;

		void throughputByConnecting(std::vector<omp_lock_t> &pixelLocks, std::vector<vec3f> &colors, 
			const Path &eyePath, const Path &lightPath);

		void throughputByMerging(std::vector<vec3f> &colors, const Path &eyePath, std::vector<LightPoint> &lightVertices);

	public:
		VCMTracer(RenderEngine *engine, uint spp, uint maxLen) :
			MCRenderer(engine, spp, maxLen)
		{
			name = "VCMTracer";
			alpha = 2.0/3;
			baseRadius = 0.01;
			radius = baseRadius;
		}
		void setRadius(const float &r) { baseRadius = r; }
		std::vector<vec3f> renderPixels();
	};
}