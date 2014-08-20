#include "mcRenderer.h"
#include "hashgrid.hxx"
#include <omp.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


namespace LFJ{
	class PhotonMap : public MCRenderer{
		HashGrid   surfaceHashGrid;
		omp_lock_t surfaceHashGridLock;
		HashGrid   volumeHashGrid;
		omp_lock_t volumeHashGridLock;
		float	   mBaseRadius;
		float	   mRadius;
		float	   mAlpha;
		int		   mPhotonsNum;
		typedef struct HashGridVertex{
			Path  *pathThePointIn;
			uint   indexInThePath;
			vec3f  position;
			Ray::PhotonType photonType;
			HashGridVertex(){
				pathThePointIn = NULL;
				photonType = Ray::OUTVOL;
				indexInThePath = -1;
			}
			vec3f GetPosition() const{ return position; }
		} LightPoint;
	public:
		std::ofstream fout;
		PhotonMap(RenderEngine *engine, uint spp, uint maxLen) :
			MCRenderer(engine, spp, maxLen)
		{
			name = "PhotonMap";
			mAlpha = 2.0/3;
			mPhotonsNum = engine->scene.mCamera.mResolution.x * engine->scene.mCamera.mResolution.y;

			//fout.open("debugPPM.txt");
		}

		void setRadius(float r) { mBaseRadius = r; }
		void throughputByDensityEstimation(vec3f &color, Path &eyeMergePath, 
			std::vector<LightPoint> &surfaceVertices, std::vector<LightPoint> &volumeVertices, bool draw = false);

		void sampleMergePath(Path &path, Ray &prevRay, uint depth) const;

		std::vector<vec3f> renderPixels();

		omp_lock_t debugPrintLock;

		double ASG_dF_dTheta(const double theta, const double phi, const double Lambda, const double Miu) const;
		double ASG_dF_dPhi(const double theta, const double phi, const double Lambda, const double Miu) const;
		double ASG_dF2_dTheta(const double theta, const double phi, const double Lambda, const double Miu) const;
		double eval_ASG(const Frame &warpFrame, const vec3f &sp_dir, const double Lambda, const double Miu ) const;

		double genEllipseRatio(const double cen, const double fast, const double slow); 

		void draw_ASG_ratios();

	};
}
