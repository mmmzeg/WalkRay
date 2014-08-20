#pragma once
#include <vector>
#include <unordered_set>
#include <fstream>
#include <algorithm>
#include "macros.h"

namespace LFJ{
	class KDTree
	{
	public:
		struct Ray{
			vec3f origin;
			vec3f direction;
		};
		struct Triangle{
			vec3ui vertexIndices;
			void* sourceInformation;
		};
		enum Strategy{ LOOP, BEST } strategy;
		class Condition{
		public:
			virtual bool legal(const Ray& ray, const Triangle& tri, const float dist) const
			{
				return true;
			}
		};
	private:
		struct Node;
		Node* root;
		unsigned maxLeafTriangleNum;
		unsigned maxDepth;
		void splitLoop(Node* node, unsigned dim, unsigned depth = 0);
		void splitBest(Node* node, unsigned depth = 0);
		void destroy(Node* node);
		float intersect(const Ray& ray, const Node* node, unsigned& triangleID, const Condition* condition) const;
		float intersect(const Ray& ray, const Triangle& tri, const Condition* condition) const;
	public:
		std::vector<vec3f>	  vertexPositionList;
		std::vector<Triangle> triangleList;
		KDTree();
		~KDTree();
		void build(Strategy strategy = BEST);
		void destroy();
		float intersect(const Ray& ray, unsigned& triangleID, const Condition* condition = NULL) const;


	};

}