#pragma once
#include "camera.h"
#include "kdtree.h"
#include "mesh.h"
#include "surfaceSampler.h"

namespace LFJ{
	class AbstractObject;

	class Scene{
	public:
		std::string name;
		Rng *rng;
		Camera mCamera;
		std::vector<AbstractObject*> objects;
		
		struct ObjSourceInfo{
			uint objID;
			uint triID;
		};
		KDTree tree;
		std::vector<KDTree> objKDTrees;
		
	public:
		Scene() : mCamera(this), lightSampler(NULL) {}
		~Scene(){
			clear();
		}
		void setRng(Rng *rng) { this->rng = rng; }
		Camera& getCamera() { return mCamera; }

		SurfaceSampler *lightSampler;
		void prepareForLightSampling();
		Ray genLightSample() const;

	public:
		void buildKDTree(bool silent = false);
		void buildObjKDTrees(bool silent = false);
		void updateSceneForMotionBlur();
		void clear();
		void treeDestroy();
		float intersect(const Ray &ray, ObjSourceInfo &objSource, const KDTree::Condition *condition = NULL) const;
		AbstractObject* findInsideObject(const Ray &ray, AbstractObject *curObj=NULL) const;
		AbstractObject* findInsideObjectWithID(const Ray &ray, int objID) const;
	};
}