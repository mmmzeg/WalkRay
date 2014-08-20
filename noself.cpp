#include "stdafx.h"
#include "noself.h"
#include "scene.h"
#include "abstractObj.h"

namespace LFJ{
	NoSelfCondition::NoSelfCondition(Scene *scene, const Ray &ray){
		this->scene = scene;
		this->curInsideObj = ray.insideObj;
		this->curContactObj = ray.contactObj;
	}

	bool NoSelfCondition::legal(const KDTree::Ray &ray, const KDTree::Triangle &tri, const float dist) const{
		AbstractObject *intersectObject = scene->objects[((Scene::ObjSourceInfo*)tri.sourceInformation)->objID];
		uint fi = ((Scene::ObjSourceInfo*)tri.sourceInformation)->triID;
		float d = ray.direction.dot(intersectObject->getWorldNormal(fi, ray.origin + ray.direction * dist));
		bool in = d <= 0, out = d >= 0;
		if(curInsideObj == curContactObj)	return !(in && curContactObj == intersectObject);
		else								return !(out && curInsideObj != intersectObject);
	}
}