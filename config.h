#pragma once
#include <io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <unordered_map>
#include <rapidxml.hpp>
#include "macros.h"
#include "nvMatrix.h"

using namespace std;
using namespace rapidxml;

namespace LFJ{
	class RenderEngine;
	class AbstractObject;
	class Mesh;

	class Config{
		RenderEngine *engine;
	public:
		Config(RenderEngine *e) : engine(e)
		{}
		void load(const string &configFilePath);
		int loadTime;

	private:
		AbstractObject *generateSceneObject(const std::string &path, xml_node<>* nodeObj, xml_node<>* nodeMat, Mesh* shape = NULL);

	private:
		string getRootPath() const{ return rootPath; }
		char *textFileRead(const char *fn);
		int textFileWrite(const char *fn, const char *s);
		mat4f readMatrix(const char *value) const;
		vec3f readVec(const char *value) const;
		xml_node<>* findNode(const string& filePath, const string& nodeTag, const string& nodeName);
		xml_node<>* findNode(xml_node<>* root, const string& nodeTag, const string& nodeName);
		pair<string, string> getPathAndName(xml_node<>* node);
		void clear();
	private:
		string rootPath;
		string currentPath;
		unordered_map<string, pair<xml_document<>*, char*>> path_doc;
	};
}