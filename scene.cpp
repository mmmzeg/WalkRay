#include "stdafx.h"
#include "scene.h"
#include "abstractObj.h"
namespace LFJ{
	void Scene::prepareForLightSampling(){
		if(lightSampler)	delete lightSampler;
		lightSampler = new SurfaceSampler(rng);
		for(uint i = 0; i < objects.size(); i++){
			if(objects[i]->isEmissive()){
				lightSampler->targetObjects.push_back(objects[i]);
			}
		}
		lightSampler->prepareForSampling();
	}

	Ray Scene::genLightSample() const{
		return lightSampler->genSample();
	}

	void Scene::buildKDTree(bool silent){
		uint vi_offset = 0;
		if(!silent)
			std::cout << "build KDTree " << " size = " << objects.size() << std::endl;
		for(uint oi = 0; oi < objects.size(); oi++){
			if(!silent)
				std::cout << "oi = " << oi << std::endl;
			AbstractObject *obj = objects[oi];
			for(uint i = 0; i < obj->getVertexNum(); i++){
				tree.vertexPositionList.push_back(obj->getWorldVertexPosition(i));
			}
			for(uint i = 0; i < obj->getTriangleNum(); i++){
				KDTree::Triangle tri;
				tri.vertexIndices = obj->getVertexIndices(i) + vec3ui(vi_offset, vi_offset, vi_offset);
				tri.sourceInformation = new ObjSourceInfo;
				((ObjSourceInfo*)tri.sourceInformation)->objID = oi;
				((ObjSourceInfo*)tri.sourceInformation)->triID = i;
				tree.triangleList.push_back(tri);
			}
			vi_offset += obj->getVertexNum();
		}
		tree.build();
		if(!silent)
			std::cout << "TREE BUILDED" << std::endl;
	}

	void Scene::buildObjKDTrees(bool silent){
		objKDTrees.resize(objects.size());
		if(!silent)
			std::cout << "build objKDTree " << " size = " << objects.size() << std::endl;
		for(uint oi = 0; oi < objects.size(); oi++){
			AbstractObject *obj = objects[oi];
			for(uint i = 0; i < obj->getVertexNum(); i++){
				objKDTrees[oi].vertexPositionList.push_back(obj->getWorldVertexPosition(i));
			}
			for(uint i = 0; i < obj->getTriangleNum(); i++){
				KDTree::Triangle tri;
				tri.vertexIndices = obj->getVertexIndices(i);
				tri.sourceInformation = new ObjSourceInfo;
				((ObjSourceInfo*)tri.sourceInformation)->objID = oi;
				((ObjSourceInfo*)tri.sourceInformation)->triID = i;
				objKDTrees[oi].triangleList.push_back(tri);
			}
			objKDTrees[oi].build();
		}
	}

	void Scene::updateSceneForMotionBlur(){
		for(int objID = 0; objID < objects.size(); objID++){
			if(objects[objID]->hasMotionBlur()){
				objects[objID]->updateTransform();
				objects[objID]->getBoundingBox(objects[objID]->minCoord, objects[objID]->maxCoord);
			}
		}

		treeDestroy();
		
		buildKDTree(true);
		buildObjKDTrees(true);
		return ;
	}
	
	void Scene::treeDestroy(){
		for(uint i = 0; i < objKDTrees.size(); i++){
			objKDTrees[i].destroy();
		}
		tree.destroy();
	}


	void Scene::clear(){
		for(uint i = 0; i < objects.size(); i++)
			delete objects[i];
		objects.clear();

		treeDestroy();
		
		if(lightSampler){
			delete lightSampler;
		}
	}

	float Scene::intersect(const Ray &ray, ObjSourceInfo &objSource, const KDTree::Condition *condition) const{
		KDTree::Ray kdRay;
		kdRay.direction = ray.direction;
		kdRay.origin = ray.origin;

		uint tid;
		float dist = tree.intersect(kdRay, tid, condition);
		if(dist >= 0){
			objSource = *(ObjSourceInfo*)(tree.triangleList[tid].sourceInformation);
		}
		return dist;
	}

	AbstractObject* Scene::findInsideObject(const Ray &ray, AbstractObject *currentObject) const{
		KDTree::Ray kdray_front, kdray_back;
		kdray_front.direction = ray.direction;
		kdray_front.origin = ray.origin;
		kdray_back.direction = -ray.direction;
		kdray_back.origin = ray.origin;
		for(int i = objKDTrees.size()-1; i>=0; i--){
			uint tid;
			vec3f normal;
			float dist;
			if(currentObject == objects[i])
				continue;
			dist = objKDTrees[i].intersect(kdray_back, tid);
			if(dist<0)
				continue;
			normal = objects[i]->getWorldNormal(tid, kdray_back.origin + kdray_back.direction*dist);
			if(normal.dot(kdray_back.direction) < 0)
				continue;

			dist = objKDTrees[i].intersect(kdray_front, tid);
			if(dist<0)
				continue;
			normal = objects[i]->getWorldNormal(tid, kdray_front.origin + kdray_front.direction*dist);
			if(normal.dot(kdray_front.direction) < 0)
				continue;

			return objects[i];
		}
		return NULL;
	}

	AbstractObject* Scene::findInsideObjectWithID(const Ray &ray, int objID) const{
		KDTree::Ray kdray_front, kdray_back;
		kdray_front.direction = ray.direction;
		kdray_front.origin = ray.origin;
		kdray_back.direction = -ray.direction;
		kdray_back.origin = ray.origin;
		int i = objID;
		{
			uint tid;
			vec3f normal;
			float dist;
			 
			dist = objKDTrees[i].intersect(kdray_back, tid);
			if(dist<0){
				return NULL;
			}
			normal = objects[i]->getWorldNormal(tid, kdray_back.origin + kdray_back.direction*dist);
			if(normal.dot(kdray_back.direction) < 0){
				return NULL;
			}

			dist = objKDTrees[i].intersect(kdray_front, tid);
			if(dist<0){
				return NULL;
			}
			normal = objects[i]->getWorldNormal(tid, kdray_front.origin + kdray_front.direction*dist);
			if(normal.dot(kdray_front.direction) < 0){
				return NULL;
			}

			return objects[i];
		}
	}

}