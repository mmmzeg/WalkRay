#pragma once
#include "abstractObj.h"
#include "phaseFunction.h"

#include <omp.h>

namespace LFJ{

	class HeterogeneousVolume : public AbstractObject{
		vec3f scatteringCoeff;
		vec3f absorptionCoeff;
		vec3f extinctionCoeff;
		typedef enum{ SCATTERING, ABSORPTION, EXTINCTION } LOOK_UP_TYPE; 
		float IOR;
		float stepSize; // for ray marching
		float densityScale;

		float scatteringScale;
		float absorptionScale;

		struct VolGridBox{
			vec3f p0, p1;
			int nx, ny, nz;
		} mBBox;
		float *mDensityMap;				// Smoke

		vec3f *mSubSurfaceDensityMap_Scattering;   // SubSurface
		vec3f *mSubSurfaceDensityMap_Absorption;   // SubSurface
		vec3f *mSubSurfaceDensityMap_Extinction;   // SubSurface

		inline float D(int x, int y, int z) const{
			x = CLAMP(x, 0, mBBox.nx-1);
			y = CLAMP(y, 0, mBBox.ny-1);
			z = CLAMP(z, 0, mBBox.nz-1);
			return mDensityMap[z*mBBox.nx*mBBox.ny + y*mBBox.nx + x] * densityScale;
		}

		bool  isSubsurface; // if subsurface, the density file will be RGB vec3f, instead of one float value.
		bool  volumeLookUpAll;
		inline vec3f SD(int x, int y, int z, LOOK_UP_TYPE type) const{
			x = CLAMP(x, 0, mBBox.nx-1);
			y = CLAMP(y, 0, mBBox.ny-1);
			z = CLAMP(z, 0, mBBox.nz-1);

			int index = z*mBBox.nx*mBBox.ny + y*mBBox.nx + x;
			vec3f densitySub;
			if(volumeLookUpAll){
				switch(type){
				case SCATTERING:
					densitySub = mSubSurfaceDensityMap_Scattering[index];	break;
				case ABSORPTION:
					densitySub = mSubSurfaceDensityMap_Absorption[index];	break;
				case EXTINCTION:
					densitySub = mSubSurfaceDensityMap_Extinction[index];	break;
				default:
					// do nothing	
					break;
				}
			}
			else{
				densitySub = mSubSurfaceDensityMap_Extinction[index];
			}
			return densitySub;
		}

		inline float lookUpDensity(const vec3f &worldPos) const;		   // Smoke
		inline vec3f lookUpSubSurfaceVolumeData(const vec3f &worldPos, LOOK_UP_TYPE type) const; // SubSurface

		int  objID;
		inline bool checkIn(const vec3f &worldPos, const int objID) const;
		inline int check(const Ray &inRay, float *intersectDist = NULL) const;
	public:

		HeterogeneousVolume(Scene *scene, float g) : 
			AbstractObject(scene), 
			mDensityMap(NULL), 
			mSubSurfaceDensityMap_Scattering(NULL),
			mSubSurfaceDensityMap_Absorption(NULL),
			mSubSurfaceDensityMap_Extinction(NULL)
		{
			bsdf = new PhaseFunction(g);
			IOR = 1.5;
			stepSize = 0.002;
			densityScale = 100;
			scatteringScale = 100;
			absorptionScale = 100;
			isSubsurface = false;
			volumeLookUpAll = true;
			objID = scene->objects.size();
		}

		~HeterogeneousVolume(){
			delete bsdf;
			if(mDensityMap)								delete mDensityMap;
			if(mSubSurfaceDensityMap_Scattering)		delete mSubSurfaceDensityMap_Scattering;
			if(mSubSurfaceDensityMap_Absorption)		delete mSubSurfaceDensityMap_Absorption;
			if(mSubSurfaceDensityMap_Extinction)		delete mSubSurfaceDensityMap_Extinction;
		}
		void loadDensityMap(const std::string &filename);
		void loadSubSurfaceVolumeData(const std::string &filename, const std::string &filename2);

		Ray scatter(Ray &inRay) const;

		vec3f evaluateBSDF(const Ray &inRay, const Ray &outRay) const;
		float evalOriginProbability(const Ray &inRay, const Ray &outRay) const;
		float evalDirectionProbability(const Ray &inRay, const Ray &outRay) const;
		vec3f randianceDecay(const Ray &inRay, const float &dist) const;
		float getIOR() const { return IOR; }
		bool isVolume() const { return true; }
		bool hasCosine() const { return false; }
		bool homogeneous() const { return false; }
	private:
		float integrateDensity(const Ray &inRay, float dist) const;
		vec3f tau(const Ray &inRay, float dist) const;
		float pMedium(const Ray &inRay, float dist) const;
		float PSurface(const Ray &inRay, float dist) const;

		float getAlbedo() const;
		float getAlbedo(const vec3f &p) const;
		bool sampleDistance(const Ray &inRay, float &distance, float &pdfSuccess, float &pdfFailure) const;
		int findDesiredIntegralDensity(const Ray &inRay, const float desiredDensity, 
			float &t, float &integratedDensity, float &densityAtMinT, float &densityAtT) const;

	public:
		void setSigma(vec3f sigmaS, vec3f sigmaA){
			scatteringCoeff = sigmaS;
			absorptionCoeff = sigmaA;
			extinctionCoeff = scatteringCoeff + absorptionCoeff;
			std::cout << "simga_s=" << scatteringCoeff << " sigma_a=" << absorptionCoeff << " simga_t= " << extinctionCoeff << std::endl;
		}
		void setDensityScale(float s){
			densityScale = s;
			std::cout << "setDensityScale=" << densityScale << std::endl;
		}
		void setScatteringScale(float s){
			scatteringScale = s;
			std::cout << "setScatteringScale=" << scatteringScale << std::endl;
		}
		void setAbsorptionScale(float a){
			absorptionScale = a;
			std::cout << "setAbsorptionScale=" << absorptionScale << std::endl;
		}
		float getDensityScale() const{
			return densityScale;
		}
		void setStepSize(float s){
			stepSize = s;
			std::cout << "setStepSize=" << stepSize << std::endl;
		}
		float getStepSize() const{
			return stepSize;
		}
		void setIOR(float ior){
			IOR = ior;
			std::cout << "setIOR=" << IOR << std::endl;
		}
		void setSubSurface(bool isSub){
			isSubsurface = isSub;
			std::cout << "setSubSurface=" << isSubsurface << std::endl;
			return ;
		}
		void setVolumeLookUp(bool lookUpAll){
			volumeLookUpAll = lookUpAll;
			std::cout << "setVolumeLookUp=" << volumeLookUpAll << std::endl;
			return ;
		}
	};
}