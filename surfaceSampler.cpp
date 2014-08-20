#include "stdafx.h"
#include "surfaceSampler.h"
#include "abstractObj.h"
#include <algorithm>

namespace LFJ{
	void SurfaceSampler::prepareForSampling(){
		for(uint i = 0; i < targetObjects.size(); i++){
			targetObjects[i]->prepareForSurfaceSampling();
			boundArea += targetObjects[i]->boundArea;
			areaThreshold.push_back(boundArea);
		}
		for(uint i = 0; i < targetObjects.size(); i++)
			targetObjects[i]->pickProb = targetObjects[i]->boundArea / boundArea;
	}
	
	Ray SurfaceSampler::genSample() const{
		float chosen = rng->genFloat() * boundArea;
		uint index = std::lower_bound(areaThreshold.begin(), areaThreshold.end(),
			chosen) - areaThreshold.begin();
		if(index == areaThreshold.size())	index--;
		return targetObjects[index]->genSurfaceSample();
	}
}