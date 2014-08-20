#include "stdafx.h"
#include "mesh.h"
using namespace std;
namespace LFJ{
	void Mesh::loadShape(const string &filename, bool normalize, vector<Mesh*> *mesh){
		bool split = mesh != NULL;
		std::vector<Mesh*> &splitedShapes = *mesh;
		int ret;
		vertexList.clear();
		faceVertexIndexList.clear();
		std::string suffix = filename.substr(filename.length()-4,4);

		std::cout << "load shape: " << "filename = " << filename <<  " suffix = " << suffix << std::endl;

		if(suffix == ".shp")
		{
			FILE* file;
			fopen_s(&file, filename.c_str(),"rb");
			int size;
			fread(&size,sizeof(int),1,file);
			vertexList.resize(size);
			fread(vertexList.data(),sizeof(float),size,file);
			fread(&size,sizeof(int),1,file);
			faceVertexIndexList.resize(size);
			fread(faceVertexIndexList.data(),sizeof(unsigned int),size,file);
			fclose(file);
		}
		else if(suffix == ".obj")
		{
			char line[BUFFERSIZE];
			char attrib[BUFFERSIZE];
			char parms[3][BUFFERSIZE];
			FILE* file;
			fopen_s(&file, filename.c_str(),"r");

			unsigned current_fi = vertexList.size();
			unsigned current_fni = vertexNormalList.size();

			Mesh* shape = NULL;

			while(fgets(line,BUFFERSIZE,file))
			{
				if(line[0] == '#')
					continue;
				int num = sscanf_s(line,"%s %s %s %s",attrib,BUFFERSIZE,parms[0],BUFFERSIZE,parms[1],BUFFERSIZE,parms[2],BUFFERSIZE);

				if(strcmp("g",attrib)==0 && split)
				{
					if(shape)
					{
						splitedShapes.push_back(shape);
					}
					shape = new Mesh;
					shape->name = parms[0];
					current_fi = vertexList.size();
					current_fni = vertexNormalList.size();
				}

				if(num!=4)
					continue;
				if(strcmp("v",attrib)==0)
				{
					vec3f vert;
					sscanf_s(parms[0],"%f",&vert.x,sizeof(float));
					sscanf_s(parms[1],"%f",&vert.y,sizeof(float));
					sscanf_s(parms[2],"%f",&vert.z,sizeof(float));
					vertexList.push_back(vert);
					if(split && shape)
						shape->vertexList.push_back(vert);
				}
				if(strcmp("f",attrib)==0)
				{
					vec3ui tri, vnTri, tTri;

					ret = sscanf_s(parms[0],"%d/%d/%d",&tri.x,&tTri.x,&vnTri.x,sizeof(unsigned));
					ret = sscanf_s(parms[1],"%d/%d/%d",&tri.y,&tTri.y,&vnTri.y,sizeof(unsigned));
					ret = sscanf_s(parms[2],"%d/%d/%d",&tri.z,&tTri.z,&vnTri.z,sizeof(unsigned));
					if(ret==1)
					{
						ret = sscanf_s(parms[0],"%d//%d",&tri.x,&vnTri.x,sizeof(unsigned));
						ret = sscanf_s(parms[1],"%d//%d",&tri.y,&vnTri.y,sizeof(unsigned));
						ret = sscanf_s(parms[2],"%d//%d",&tri.z,&vnTri.z,sizeof(unsigned));
					}
					tri -= vec3ui(1,1,1);
					vnTri -= vec3ui(1,1,1);
					faceVertexIndexList.push_back(tri);
					if(split && shape)
						shape->faceVertexIndexList.push_back(tri - vec3ui(current_fi, current_fi, current_fi));
					if(ret >= 2)
					{
						faceVertexNormalIndexList.push_back(vnTri);
						if(split && shape)
							shape->faceVertexNormalIndexList.push_back(vnTri - vec3ui(current_fni, current_fni, current_fni));
					}
				}
				if(strcmp("vn",attrib)==0)
				{
					vec3f vn;
					sscanf_s(parms[0],"%f",&vn.x,sizeof(float));
					sscanf_s(parms[1],"%f",&vn.y,sizeof(float));
					sscanf_s(parms[2],"%f",&vn.z,sizeof(float));
					vertexNormalList.push_back(vn);
					if(split && shape)
						shape->vertexNormalList.push_back(vn);
				}
			}
			fclose(file);
			if(split)
			{
				if(shape)
				{
					splitedShapes.push_back(shape);
				}
			}
		}

		if(normalize)
		{
			matrix4<float> unitizeMat = this->unitize();
			for(unsigned i=0; split && i<splitedShapes.size(); i++)
			{
				splitedShapes[i]->setTransform(unitizeMat);
				splitedShapes[i]->applyTransform();
				splitedShapes[i]->setTransform(transform);
				splitedShapes[i]->getBoundingBox(splitedShapes[i]->minCoord, splitedShapes[i]->maxCoord);
			}

		}
		getBoundingBox(minCoord, maxCoord);
	}

	void Mesh::getBoundingBox(vec3f &minCoord, vec3f &maxCoord){
		if(vertexList.size()<1)
			return;
		minCoord.x=maxCoord.x=vertexList[0].x;
		minCoord.y=maxCoord.y=vertexList[0].y;
		minCoord.z=maxCoord.z=vertexList[0].z;
		for(unsigned i=1;i<getVertexNum();i++)
		{
			float &x = vertexList[i].x;
			float &y = vertexList[i].y;
			float &z = vertexList[i].z;
			minCoord.x = MIN(x,minCoord.x);
			maxCoord.x = MAX(x,maxCoord.x);
			minCoord.y = MIN(y,minCoord.y);
			maxCoord.y = MAX(y,maxCoord.y);
			minCoord.z = MIN(z,minCoord.z);
			maxCoord.z = MAX(z,maxCoord.z);
		}
	}

	mat4f Mesh::unitize(){
		vec3f minCoord, maxCoord;
		getBoundingBox(minCoord,maxCoord);
		vec3f center = (minCoord+maxCoord)/2;
		vec3f delta = maxCoord - minCoord;
		float maxLen = max(delta.x,delta.y);
		maxLen = max(maxLen,delta.z);
		for(unsigned i=0; i<getVertexNum(); i++)
		{
			vertexList[i] -= center;
			vertexList[i] /= maxLen/2;
		}
		return mat4f(2/maxLen, 0, 0, -2/maxLen*center.x, 0, 2/maxLen, 0, -2/maxLen*center.y, 0, 0, 2/maxLen, -2/maxLen*center.z, 0, 0, 0, 1);
	}

	void Mesh::applyTransform(){
		mat4f normalMat = transpose(inverse(transform));
		for(uint i=0; i<vertexList.size(); i++)
			vertexList[i] = vec3f(transform*vec4f(vertexList[i], 1));
		for(uint i=0; i<vertexNormalList.size(); i++)
			vertexNormalList[i] = vec3f(normalMat*vec4f(vertexNormalList[i], 0));
		transform.make_identity();
	}

}