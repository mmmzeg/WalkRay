#include "stdafx.h"
#include "bidirPathTracer.h"
#include "noself.h"
#include "renderEngine.h"
#include <limits>
using namespace std;


namespace LFJ{
	std::vector<vec3f> BidirectionalPathTracer::renderPixels(){
		Camera &camera = this->engine->scene.mCamera;
		uint width = camera.mResolution.x, height = camera.mResolution.y;
		std::vector<vec3f> pixelColors(width*height, vec3f(0,0,0));

		std::vector<omp_lock_t> pixelLocks(pixelColors.size());
		for(uint i = 0; i < pixelLocks.size(); i++){
			omp_init_lock(&pixelLocks[i]);
		}

		prepareForLightSampling();

		for(uint s = 0; s < spp; s++){
			std::cout << "iteration : " << s << std::endl;

			engine->scene.updateSceneForMotionBlur();

			std::vector<vec3f> oneIterColors(pixelColors.size(), vec3f(0,0,0));
#pragma omp parallel for
			for(int p = 0; p < pixelColors.size(); p++){		
				Path eyePath, lightPath;
				Ray lightRay = genLightSample();
				samplePath(eyePath, camera.generateRay(p));
				samplePath(lightPath, lightRay);

				throughputByConnecting(pixelLocks, oneIterColors, eyePath, lightPath);
			}
#pragma omp parallel for
			for(uint i = 0; i < pixelColors.size(); i++){
				pixelColors[i] *= s / float(s + 1);
				if(!isLegalColor(oneIterColors[i]))
					oneIterColors[i] = vec3f(0,0,0);
				pixelColors[i] += camera.fixVignetting(oneIterColors[i], i) / (s + 1);
			}

			camera.mFilm.setBuffer(pixelColors);
			std::string filename = engine->renderer->name + engine->scene.name + ".pfm";
			camera.mFilm.savePFM(filename);
		}
		return pixelColors;
	}

	void BidirectionalPathTracer::throughputByConnecting(std::vector<omp_lock_t> &pixelLocks,
		std::vector<vec3f> &colors, const Path &eyePath, const Path &lightPath)
	{
		Camera &camera = this->engine->scene.mCamera;
		uint width = camera.mResolution.x, height = camera.mResolution.y;

		// len w.r.t vertexs number of a path, not edges
		int maxEyeLen = eyePath.size(), maxLightLen = lightPath.size(),
			maxWholeLen = maxEyeLen + maxLightLen;

		for(int wholePathLen = 2; wholePathLen <= maxWholeLen; wholePathLen++){
			Path wholePath(wholePathLen);

			// calculate the MinMax light sub-path len under the wholePathLen
			int minSubLightLen = wholePathLen - maxEyeLen > 0 ? wholePathLen-maxEyeLen : 0;
			int maxSubLightLen = wholePathLen - 1 >= maxLightLen ? maxLightLen : wholePathLen-1;

			for(int lightLen = minSubLightLen; lightLen <= maxSubLightLen; lightLen++){
				int eyeLen = wholePathLen - lightLen;
				// construt a whole connected path
				for(int i = 0; i < lightLen; i++)	wholePath[i] = lightPath[i];
				for(int i = 0; i < eyeLen; i++)		wholePath[lightLen+i] = eyePath[eyeLen-i-1];

				int lightConnectID = lightLen - 1, eyeConnectID = lightLen;
				Ray &lightConnectRay = wholePath[lightConnectID], &eyeConnectRay = wholePath[eyeConnectID];

				// whole path must start from a light source
				if(!(wholePath.front().contactObj && wholePath.front().contactObj->isEmissive()))
					continue;

				// we've sampled a classical PT path, use this sample only if the path can't be connected
				if(lightLen == 0 && !pathCanNotConnect(wholePath))
					continue;

				// situation with a vertex connection 
				if(eyeLen > 0 && lightLen > 0){
					// more than one light source in this path
					if(lightLen > 1 && (lightConnectRay.contactObj && lightConnectRay.contactObj->isEmissive()))
						continue;
					// eye sub-path hit light
					if(eyeConnectRay.contactObj && eyeConnectRay.contactObj->isEmissive())
						continue;
					// create "connect edge"
					if(!connectEdge(wholePath, lightConnectID))
						continue;
					// test the visibility of the two vertices of "connect edge" 
					if(!visibilityTest(eyeConnectRay, lightConnectRay))
						continue;
				}

				// calculate the color & probability of the connected path
				vec4f packColorProb = connectingColorProb(wholePath, lightConnectID);
				if(packColorProb.w == 0)				continue;
				if(vec3f(packColorProb).length() == 0)	continue;

				// calculate MIS weight of the connected path, expTerm=1(balanced strategy) expTerm=2(power strategy) 
				float misWeight = connectingMisWeight(wholePath, lightConnectID, 1);

				vec3f contrib = vec3f(packColorProb) / packColorProb.w * misWeight;
				
				Ray &cameraRay = wholePath.back();

				if(eyeLen > 1){
					omp_set_lock(&pixelLocks[cameraRay.pixelID]);
					colors[cameraRay.pixelID] += contrib;
					omp_unset_lock(&pixelLocks[cameraRay.pixelID]);
				}
				else{
					vec2f rasterCoord = camera.useDOF() ?
						camera.convToRaster2(wholePath[wholePath.size()-2].origin, cameraRay.origin) :
						camera.convToRaster(wholePath[wholePath.size()-2].origin);
					
					if(camera.checkPixelBound(rasterCoord))
					{
						int x = rasterCoord.x, y = rasterCoord.y;
						float solidAngle = camera.useDOF() ? 
							camera.computeSolidAngle(wholePath[wholePath.size()-2].origin, cameraRay.origin) :
							camera.getPixelSolidAngle(y*width + x);
						float imageToSolidAngleFactor = 1.0 / solidAngle;
						contrib *= (imageToSolidAngleFactor / colors.size());	
						omp_set_lock(&pixelLocks[y*width + x]);
						colors[y*width + x] += contrib;
						omp_unset_lock(&pixelLocks[y*width + x]);
					}
				}
			}
		}
		return ;
	}

	float BidirectionalPathTracer::connectingMisWeight(const Path& connectedPath, 
		int connectLightID, float expTerm) const
	{
		Camera &camera = this->engine->scene.mCamera;
		uint width = camera.mResolution.x, height = camera.mResolution.y;

		std::vector<double> directPathProb(connectedPath.size());
		std::vector<double> reversePathProb(connectedPath.size());
		double allTechPathProb = 0;

		// connectedPath:	 Eye---------------------Light
		//           say,     V4    V3    V2    V1    V0  (direction: <-)  
		std::vector<float> distRecord;
		directPathProb.front() = connectedPath.front().originProb;//connectedPath.front().evalOriginProbability(connectedPath.front());//connectedPath.front().originProb;//
		for(int i = 1; i < connectedPath.size(); i++){
			float dist = MAX((connectedPath[i].origin - connectedPath[i-1].origin).length(), EPSILON);
			distRecord.push_back(dist);
			Ray linkRay = linkVertex(connectedPath, connectLightID, i, true);
			float cosThere = linkRay.cosineTerm();
			directPathProb[i] = directPathProb[i-1] * PdfWtoA(1.0, dist, cosThere);
			if(connectedPath[i-1].isDeltaDirection)
				continue;
			float linkOriProb = linkVertex(connectedPath, connectLightID, i-1, false).evalOriginProbability(
				linkVertex(connectedPath, connectLightID, i, false)), linkDirProb;
			if(i > 1){
				linkDirProb = linkVertex(connectedPath, connectLightID, i-2, false).evalDirectionProbability(
					linkVertex(connectedPath, connectLightID, i-1, false));
			}
			else{
				Ray ray = linkVertex(connectedPath, connectLightID, i-1, false);
				linkDirProb = ray.evalDirectionProbability(ray);
			}
			directPathProb[i] *= linkDirProb * linkOriProb;
		}

		// connectedPath:	 Eye---------------------Light
		//           say,     V4    V3    V2    V1    V0  (direction: ->)  
		reversePathProb.back() = connectedPath.back().originProb;//evalOriginProbability(connectedPath.back());//.originProb;
		for(int i = connectedPath.size()-2; i >= 0; i--){
			float dist = distRecord[i];
			Ray linkRay = linkVertex(connectedPath, connectLightID, i, false);
			float cosThere = linkRay.cosineTerm();
			reversePathProb[i] = reversePathProb[i+1] * PdfWtoA(1.0, dist, cosThere);
			if(connectedPath[i+1].isDeltaDirection)
				continue;
			float linkOriProb = linkVertex(connectedPath, connectLightID, i+1, true).evalOriginProbability(
				linkVertex(connectedPath, connectLightID, i, true)), linkDirProb;
			if(i < connectedPath.size()-2){
				linkDirProb = linkVertex(connectedPath, connectLightID, i+2, true).evalDirectionProbability(
					linkVertex(connectedPath, connectLightID, i+1, true));
			}
			else{
				Ray ray = linkVertex(connectedPath, connectLightID, i+1, true);
				linkDirProb = ray.evalDirectionProbability(ray);
			}
			reversePathProb[i] *= linkDirProb * linkOriProb;
		}

		for(int i = 0; i < connectedPath.size()-1; i++){
			if(connectedPath[i].isDeltaDirection==false && connectedPath[i+1].isDeltaDirection==false){
				double p = directPathProb[i] * reversePathProb[i+1];
				/*if(i == connectedPath.size()-2)ll
					p *= width * height;*/
				allTechPathProb += pow(p, double(expTerm));
			}
		}

		if(pathCanNotConnect(connectedPath))
			allTechPathProb += pow(reversePathProb.front(), double(expTerm));

		double selfProb = connectLightID == -1 ? reversePathProb.front() : directPathProb[connectLightID] * reversePathProb[connectLightID+1];

		double weight = pow(selfProb, double(expTerm)) / allTechPathProb;

		return MAX(weight, 0);
	}


	/* \brief: Link vertexs used for calculating path probility.

	Return Ray ***starts from Vi to Vj***
	*/
	Ray BidirectionalPathTracer::linkVertex(const Path &path, int connectLightID, int startID, bool reverse) const{
		int endID = reverse ? startID-1 : startID+1;
		const Ray &startRay = path[CLAMP(startID,0,path.size()-1)];
		const Ray &endRay   = path[CLAMP(endID,0,path.size()-1)];
		Ray linkRay = startRay;
		if((reverse && startID>connectLightID) || (!reverse && startID<=connectLightID))
			return linkRay;
		linkRay.direction = endRay.origin - startRay.origin;
		linkRay.direction.normalize();
		linkRay.insideObj = endRay.insideObj;
		return linkRay;
	}

	//////////////////////////////////////////////////DONE/////////////////////////////////////////////////////////
	/* \brief: Check whether the path given is able to be connected

	Return true if no two consecutive vertex that are non-specular,
	false otherwise
	*/
	bool BidirectionalPathTracer::pathCanNotConnect(const Path &path) const{
		for(int i = 0; i < path.size() - 1; i++){
			if(!path[i].isDeltaDirection && !path[i+1].isDeltaDirection)
				return false;
		}
		return true;
	}

	/* \brief: Used for vertex connection, calculate the color & prob

	connectedPath: a path connected by one eye sub-path and one light-sub path
	connectLightID: Ray index of the connect position, 
	given a connectedPath, say, y0y1y2..yL - xE..x2x1x0 (y0 is light, x0 is camera)
	then the parameter is equal to L
	*/
	vec4f BidirectionalPathTracer::connectingColorProb(const Path &connectedPath, int connectLightID) const{
		vec3f color(1,1,1);		float prob = 1;
		for(int i = 0; i < connectedPath.size(); i++){
			color *= connectedPath[i].radiance;
			float dist;
			if(i <= connectLightID){
				dist = MAX((connectedPath[i].origin - connectedPath[i+1].origin).length(), EPSILON);
				color *= connectedPath[i].radianceDecay(dist);
			}
			else if(i > connectLightID+1){
				dist = MAX((connectedPath[i-1].origin - connectedPath[i].origin).length(), EPSILON);
				color *= connectedPath[i].radianceDecay(dist);
			}
			if(i == connectLightID && i < connectedPath.size()-1){
				color *= connectedPath[i].cosineTerm() * connectedPath[i+1].cosineTerm() / (dist * dist);
			}
			if(i != connectLightID && i != connectLightID+1){
				color *= connectedPath[i].cosineTerm();
			}
			prob *= connectedPath[i].directionProb * connectedPath[i].originProb;
		}
	
		return vec4f(color, prob);
	}

	/* \brief: create a "connect edge" when using vertex connection */
	bool BidirectionalPathTracer::connectEdge(Path &path, int connectLightID){
		Ray &lightRay = path[connectLightID];
		Ray &eyeRay = path[connectLightID + 1];
		if(lightRay.isDeltaDirection || eyeRay.isDeltaDirection)
			return false;

		const Ray &preLightRay = path[MAX(connectLightID - 1, 0)];
		const Ray &preEyeRay = path[MIN(connectLightID + 2, path.size() - 1)];

		lightRay.direction = eyeRay.origin - lightRay.origin;
		lightRay.direction.normalize();
		eyeRay.direction = -lightRay.direction;

		lightRay.directionProb = 1;
		lightRay.isDeltaDirection = false;
		lightRay.radiance = preLightRay.evalBSDF(lightRay);
		if(lightRay.contactObj && connectLightID > 0)
			lightRay.insideObj = lightRay.contactNormal().dot(lightRay.direction) > 0 ? 
			preLightRay.insideObj : lightRay.contactObj;

		eyeRay.directionProb = 1;
		eyeRay.isDeltaDirection = false;
		eyeRay.radiance = preEyeRay.evalBSDF(eyeRay);
		if(eyeRay.contactObj && connectLightID < path.size() - 2)
			eyeRay.insideObj = eyeRay.contactNormal().dot(eyeRay.direction) > 0 ?
			preEyeRay.insideObj : eyeRay.contactObj;

		return true;
	}

}