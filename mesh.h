#pragma once
#include <string>
#include <vector>
#include "util.h"
#include "frame.h"
#include "rng.h"

namespace LFJ{
	class Mesh{
	public:
		Rng *rng;
		std::string name;
		std::vector<vec3f> vertexList;					// v
		std::vector<vec3f> vertexNormalList;			// vn
		std::vector<vec3ui> faceVertexIndexList;			// f
		std::vector<vec3ui> faceVertexNormalIndexList;	// fn

		vec3f minCoord, maxCoord;
		mat4f transform;

	public:
		Mesh()
		{}
		Mesh(const std::string &filename, bool normalize = true){
			loadShape(filename, normalize);
		}
		void setRng(Rng *rng){
			this->rng = rng;
		}
		mat4f unitize();
		void setTransform(const mat4f &mat) { transform = mat; }
		void applyTransform();
		void getBoundingBox(vec3f &minCoord, vec3f &maxCoord);
		void loadShape(const std::string &filename, 
			bool normalize = true, std::vector<Mesh*> *mesh = NULL);

	public:
		void copyMesh(const Mesh &mesh){
			vertexList.assign(mesh.vertexList.begin(), mesh.vertexList.end());
			vertexNormalList.assign(mesh.vertexNormalList.begin(), mesh.vertexNormalList.end());
			faceVertexIndexList.assign(mesh.faceVertexIndexList.begin(), mesh.faceVertexIndexList.end());
			faceVertexNormalIndexList.assign(mesh.faceVertexNormalIndexList.begin(), mesh.faceVertexNormalIndexList.end());
			transform = mesh.transform;
			minCoord = mesh.minCoord;	
			maxCoord = mesh.maxCoord;
		}
		virtual vec3f getWorldNormal(uint fi, const vec3f &position, bool flat = false) const{
			vec3f vps[3], vns[3];
			for(uint i=0; i<3; i++)
				vps[i] = getWorldVertexPosition(faceVertexIndexList[fi][i]);
			mat4f normalMat = transpose(inverse(transform));
			vec3f b1 = vps[1] - vps[0];
			vec3f b2 = vps[2] - vps[0];
			vec3f faceNormal = b1.cross(b2);
			faceNormal.normalize();
			if(flat)
				return faceNormal;
			for(uint i=0; i<3; i++){
				if(vertexNormalList.size()){
					vns[i] = vertexNormalList[faceVertexNormalIndexList[fi][i]];
					vns[i] = vec3f(normalMat * vec4f(vns[i], 0));
				}
				else{
					return faceNormal;
				}
			}
			vec3f v = position - vps[0];
			float d12 = b1.dot(b2);
			float l1 = b1.length();
			float l2 = b2.length();
			float u2 = (v.dot(b1)*d12 - v.dot(b2)*l1*l1) / (d12*d12 - l1*l1*l2*l2);
			float u1 = (v.dot(b1)-u2*d12)/(l1*l1);
			vec3f normal = (1-u1-u2)*vns[0] + u1*vns[1] + u2*vns[2];
			normal.normalize();

			return normal;
		}
		vec3f getCenter() const{ return (minCoord + maxCoord) / 2; }
		float getDiagLen() const{ return (maxCoord - minCoord).length(); }
		float getTriangleArea(uint ti) const{
			vec3f vps[3];
			for(uint i = 0; i < 3; i++)
				vps[i] = getWorldVertexPosition(faceVertexIndexList[ti][i]);
			vec3f l0 = vps[1] - vps[0];
			vec3f l1 = vps[2] - vps[1];
			float c = l0.cross(l1).length();
			return c/2;
		}
		vec3f genRandTrianglePosition(uint ti) const{
			vec3f vps[3];
			for(uint i = 0; i < 3; i++)
				vps[i] = getWorldVertexPosition(faceVertexIndexList[ti][i]);
			vec3f b1 = vps[1] - vps[0];
			vec3f b2 = vps[2] - vps[0];
			float u1 = rng->genFloat(), u2 = rng->genFloat();
			if(u1 + u2 > 1){
				u1 = 1 - u1;
				u2 = 1 - u2;
			}
			return vps[0]+b1*u1+b2*u2;
		}
		uint getVertexNum() const{ return vertexList.size(); }
		uint getTriangleNum() const { return faceVertexIndexList.size(); }
		vec3f getVertexPosition(uint vi) const { return vertexList[vi]; }
		vec3ui getVertexIndices(uint ti) const { return faceVertexIndexList[ti]; }
		vec3f getWorldVertexPosition(uint vi) const { return vec3f(transform * vec4f(vertexList[vi], 1)); }

	};
}
