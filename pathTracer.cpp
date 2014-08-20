#include "stdafx.h"
#include "pathTracer.h"
#include "noself.h"
#include "renderEngine.h"
#include <limits>
namespace LFJ{
	std::vector<vec3f> PathTracer::renderPixels(){
		Camera &camera = getCamera();
		uint width = camera.mResolution.x, height = camera.mResolution.y;
		std::vector<vec3f> pixelColors(width * height, vec3f(0,0,0));
		
		if(useNextEventEstimation)
			prepareForLightSampling();

		for(uint s = 0; s < spp; s++){
			std::cout << "iteration : " << s << std::endl;
			
			engine->scene.updateSceneForMotionBlur();

#pragma omp parallel for
			for(int p = 0; p < pixelColors.size(); p++){
				Path eyePath;
				samplePath(eyePath, camera.generateRay(p));
				pixelColors[p] *= s / float(s+1);
				vec3f color = vec3f(1,1,1);

				bool hasToConnect = eyePath[eyePath.size()-1].radiance.length() <= 0;

				if(!useNextEventEstimation || !hasToConnect){
					for(int i = 0; i < eyePath.size(); i++){
						if(i != eyePath.size() - 1){
							color *= eyePath[i].cosineTerm();
							float dist = (eyePath[i+1].origin - eyePath[i].origin).length();
							color *= eyePath[i].radianceDecay(dist);
						}
						color *= eyePath[i].radiance / eyePath[i].originProb / eyePath[i].directionProb;
					}
				}
				else{
					int endIndex = (eyePath.back().contactObj || eyePath.back().insideObj) ? -1 : eyePath.size()-2;
					if(endIndex <= 0)											continue;

					Ray &endRay = eyePath[endIndex], lightRay = genLightSample();
					if(endRay.contactObj && endRay.contactObj->isEmissive())	continue;
					if(endRay.contactObj && !endRay.contactObj->nonSpecular())	continue;

					endRay.direction = lightRay.origin - endRay.origin;
					endRay.direction.normalize();
					lightRay.direction = -endRay.direction;

					float connectDist = MAX((lightRay.origin - endRay.origin).length(), EPSILON);
					
					if(endRay.direction.dot(lightRay.contactNormal()) >= 0)		continue;
					
					if(!visibilityTest(endRay, lightRay))						continue;
					
					for(int i = 0; i < endIndex; i++){
						color *= eyePath[i].radiance / eyePath[i].originProb / eyePath[i].directionProb;
						color *= eyePath[i].cosineTerm();
						float dist = (eyePath[i+1].origin - eyePath[i].origin).length();
						color *= eyePath[i].radianceDecay(dist);
					}
					color *= eyePath[endIndex-1].evalBSDF(endRay) * lightRay.radiance * endRay.radianceDecay(connectDist)
						* lightRay.cosineTerm() * endRay.cosineTerm() / (connectDist * connectDist);
					color /= eyePath[endIndex].originProb * lightRay.originProb;
				}
				if(!isLegalColor(color))
					color = vec3f(0,0,0);
				pixelColors[p] += camera.fixVignetting(color, p) / (s+1);
			}
			camera.mFilm.setBuffer(pixelColors);
			std::string filename = engine->renderer->name + engine->scene.name + ".pfm";
			camera.mFilm.savePFM(filename);
		}
		return pixelColors;
	}
}