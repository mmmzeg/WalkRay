#pragma once
#include "ray.h"
#include "kdtree.h"

namespace LFJ{
	class Scene;

	class NoSelfCondition : public KDTree::Condition{
	public:
		Scene *scene;
		AbstractObject *curInsideObj;
		AbstractObject *curContactObj;

		NoSelfCondition() {}
		NoSelfCondition(Scene *scene, const Ray &ray);
		bool legal(const KDTree::Ray &ray, const KDTree::Triangle &tri, const float dist) const;
	};
}