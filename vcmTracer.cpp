#include "stdafx.h"
#include "vcmTracer.h"
#include <limits>
using namespace std;
namespace LFJ{
	std::vector<vec3f> VCMTracer::renderPixels(){
		Camera &camera = this->engine->scene.mCamera;
		uint width = camera.mResolution.x, height = camera.mResolution.y;

		std::vector<vec3f> pixelColors(width*height, vec3f(0,0,0));
		std::vector<omp_lock_t> pixelLocks(pixelColors.size());
		for(uint i = 0; i < pixelLocks.size(); i++){
			omp_init_lock(&pixelLocks[i]);
		}
		omp_init_lock(&hashGridLock);

		prepareForLightSampling();

		radius = baseRadius;

		for(uint s = 0; s < spp; s++){
			std::cout << "iteration : " << s << std::endl;

			engine->scene.updateSceneForMotionBlur();

			std::vector<vec3f> oneIterColors(pixelColors.size(), vec3f(0,0,0));

			radius = MAX(baseRadius * sqrt(powf(s+1, alpha-1)), EPSILON);

			std::vector<Path*> pixelLightPaths(pixelColors.size(), NULL); 
			std::vector<LightPoint> lightVertices(pixelColors.size()); 

			hashGrid.Reserve(pixelColors.size());

#pragma omp parallel for
			// step1 : sample light paths and build range search struct
			for(int p = 0; p < pixelColors.size(); p++){
				Ray lightRay = genLightSample();
				pixelLightPaths[p] = new Path;
				Path &lightPath = *pixelLightPaths[p];
				samplePath(lightPath, lightRay);
				for(int i = 1; i < lightPath.size(); i++){
					// light is not reflective
					if(lightPath[i].contactObj && lightPath[i].contactObj->isEmissive())
						break;
					// only store particles non-specular
					if(lightPath[i].isDeltaDirection)
						continue;
					LightPoint lightPoint;
					lightPoint.position = lightPath[i].origin;
					lightPoint.indexInThePath = i;
					lightPoint.pathThePointIn = &lightPath;
					omp_set_lock(&hashGridLock);
					lightVertices.push_back(lightPoint);
					omp_unset_lock(&hashGridLock);
				}
			}
			hashGrid.Build(lightVertices, radius);

			// step2 : calculate pixel colors by vertex connect and vertex merge
#pragma omp parallel for
			for(int p = 0; p < pixelColors.size(); p++){
				const Path &lightPath = *pixelLightPaths[p];
				Path eyePath;
				samplePath(eyePath, camera.generateRay(p));

				throughputByConnecting(pixelLocks, oneIterColors, eyePath, lightPath);

				throughputByMerging(oneIterColors, eyePath, lightVertices);
			}
#pragma omp parallel for
			for(uint i = 0; i < pixelColors.size(); i++){
				pixelColors[i] *= s / float(s+1);
				if(!isLegalColor(oneIterColors[i]))
					oneIterColors[i] = vec3f(0,0,0);
				pixelColors[i] += camera.fixVignetting(oneIterColors[i], i) / (s + 1);
				delete pixelLightPaths[i];
			}
			camera.mFilm.setBuffer(pixelColors);
			std::string filename = engine->renderer->name + engine->scene.name + ".pfm";
			camera.mFilm.savePFM(filename);
		}
		return pixelColors;
	}

	void VCMTracer::throughputByConnecting(std::vector<omp_lock_t> &pixelLocks, std::vector<vec3f> &colors, 
		const Path &eyePath, const Path &lightPath)
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
					if(!connectEdge(wholePath, lightConnectID, VC))
						continue;
					// test the visibility of the two vertices of "connect edge" 
					if(!visibilityTest(eyeConnectRay, lightConnectRay))
						continue;
				}

				// calculate the color & probability of the connected path
				vec4f packColorProb = connectingColorProb(wholePath, lightConnectID, VC);
				if(packColorProb.w == 0)				continue;
				if(vec3f(packColorProb).length() == 0)	continue;

				// calculate MIS weight of the connected path, expTerm=1(balanced strategy) expTerm=2(power strategy) 
				float misVCWeight = vcmMisWeight(wholePath, lightConnectID, VC);

				vec3f contrib = vec3f(packColorProb) / packColorProb.w * misVCWeight;
			
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

	void VCMTracer::throughputByMerging(std::vector<vec3f> &colors, const Path &eyePath, std::vector<LightPoint> &lightVertices){
		class Query{
			VCMTracer *vcmTracer;
			vec3f contrib;
			const Path &eyePath;
			int eyePathLen;
			float radius;
		public:
			Query(VCMTracer *tracer, const Path &path, const float &r) : vcmTracer(tracer), eyePath(path), radius(r) {}
			void SetContrib(const vec3f &color) { contrib = color; }
			vec3f GetContrib() const { return contrib; }
			void SetEyePathLen(const int &len) { eyePathLen = len; }
			vec3f GetPosition() const { return eyePath[eyePathLen-1].origin; }
			void Process(const LightPoint &lightPoint){
				Path connectedPath;
				if(!lightPoint.pathThePointIn || lightPoint.indexInThePath < 0)
					return ;
				connectedPath.assign(lightPoint.pathThePointIn->begin(), lightPoint.pathThePointIn->begin() + lightPoint.indexInThePath);
				for(int i = 0; i < eyePathLen; i++)
					connectedPath.push_back(eyePath[eyePathLen-i-1]);
				if(!vcmTracer->connectEdge(connectedPath, lightPoint.indexInThePath-1, VM))
					return ;
				vec4f packColorProb = vcmTracer->connectingColorProb(connectedPath, lightPoint.indexInThePath-1, VM);
				if(vec3f(packColorProb).length()==0 || packColorProb.w==0)
					return ;
				float misVMWeight = vcmTracer->vcmMisWeight(connectedPath, lightPoint.indexInThePath-1, VM);
				contrib += misVMWeight * vec3f(packColorProb) / packColorProb.w;
			}

			void EllipseProcess(const LightPoint &lightPoint, const Frame *ellipseFrame, const vec3f &hitPoint, float aniRatio){
				// only used in Photon Mapping method.
			}

		};

		int nVM = engine->scene.mCamera.mResolution.x * engine->scene.mCamera.mResolution.y;
		Query query(this, eyePath, this->radius);
		for(int i = 1; i < eyePath.size(); i++){
			if(eyePath[i].contactObj && eyePath[i].contactObj->isEmissive())
				break;
			if(eyePath[i].isDeltaDirection)
				continue;
			query.SetContrib(vec3f(0,0,0));
			query.SetEyePathLen(i + 1);
		//	omp_set_lock(&hashGridLock);
			hashGrid.Process(lightVertices, query);
		//	omp_unset_lock(&hashGridLock);
			colors[eyePath.front().pixelID] += query.GetContrib() / nVM;
		}
		return ;
	}

	/* \brief: VCM mis weight function.

	Tech rep: 
	https://graphics.cg.uni-saarland.de/fileadmin/cguds/papers/2012/georgiev_tr2012/georgiev_tr2012_rev1.pdf
	*/
	float VCMTracer::vcmMisWeight(const Path& connectedPath, int connectLightID, VCMType vcmType, float expTerm) const{
		Camera &camera = this->engine->scene.mCamera;
		uint width = camera.mResolution.x, height = camera.mResolution.y;

		std::vector<double> directPathProb(connectedPath.size());
		std::vector<double> reversePathProb(connectedPath.size());
		double allTechPathProb = 0;

		// connectedPath:	 Eye---------------------Light
		//           say,     V4    V3    V2    V1    V0  (direction: <-)  
		//                      dist3 dist2 dist1 dist0
		std::vector<float> distRecord;
		directPathProb.front() = connectedPath.front().originProb;//connectedPath.front().evalOriginProbability(connectedPath.front());
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

		// calculate VCM mis weight.
		const int nVC = 1, nVM = width * height;
		double myTechProb = connectLightID == -1 ? reversePathProb.front() : directPathProb[connectLightID] * reversePathProb[connectLightID+1];
		for(int i = 0; i < connectedPath.size()-1; i++){
			// VC 
			if(connectedPath[i].isDeltaDirection==false && connectedPath[i+1].isDeltaDirection==false){
				double p = directPathProb[i] * reversePathProb[i+1];
				/*if(i == connectedPath.size()-2)
					p *= width * height;*/
				allTechPathProb += pow(p, double(expTerm)) * pow(nVC, double(expTerm));
			}
			// VM
			if(i < connectedPath.size()-2 && !connectedPath[i+1].isDeltaDirection){
				double p = directPathProb[i] * reversePathProb[i+1];
				if(!connectedPath[i].isDeltaDirection){
					if(i > 0){
						p *= linkVertex(connectedPath, connectLightID, i-1, false).evalDirectionProbability(
							linkVertex(connectedPath, connectLightID, i, false));
					}
					else{
						Ray ray = linkVertex(connectedPath, connectLightID, i, false);
						p *= ray.evalDirectionProbability(ray);
					}
				}
				p *= linkVertex(connectedPath, connectLightID, i, false).evalOriginProbability(
					linkVertex(connectedPath, connectLightID, i+1, false));
				float cosThere = linkVertex(connectedPath, connectLightID, i+1, true).cosineTerm();
				p *= PdfWtoA(1.0, distRecord[i], cosThere);
				p *= PI * radius * radius;
				if(connectedPath[i+1].insideObj && !connectedPath[i+1].contactObj)
					p *= 4.0/3 * radius;
				if(vcmType == VM && i == connectLightID)
					myTechProb = p;

				allTechPathProb += pow(p, double(expTerm)) * pow(nVM, double(expTerm));
			}
		}

		if(pathCanNotConnect(connectedPath))
			allTechPathProb += pow(reversePathProb.front(), double(expTerm));

		myTechProb *= vcmType == VC ? nVC : nVM;

		if(!(pow(myTechProb, double(expTerm)) / allTechPathProb >= 0))
			return 0;

		return pow(myTechProb, double(expTerm)) / allTechPathProb;
	}

	/* \brief: Used for vertex connection, calculate the color & prob

	connectedPath: a path connected by one eye sub-path and one light-sub path
	connectLightID: Ray index of the connect position, 
	given a connectedPath, say, y0y1y2..yL - xE..x2x1x0 (y0 is light, x0 is camera)
	then the parameter is equal to L
	*/
	vec4f VCMTracer::connectingColorProb(const Path &connectedPath, int connectLightID, VCMType vcmType) const{
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
			if(vcmType == VM && i == connectLightID){
				prob *= PI * radius * radius;
				float cosThere = linkVertex(connectedPath, connectLightID, i+1, true).cosineTerm();
				prob *= PdfWtoA(1.0, dist, cosThere);
				Ray prevRay = linkVertex(connectedPath, connectLightID, i, false);
				Ray nextRay = linkVertex(connectedPath, connectLightID, i+1, false);
				prob *= prevRay.evalOriginProbability(nextRay);
				if(connectedPath[i+1].insideObj && !connectedPath[i+1].contactObj){
					prob *= 4.0/3 * radius;
				}
			}
		}
		return vec4f(color, prob);
	}

	/* \brief: create a "connect edge" when using vertex connection */
	bool VCMTracer::connectEdge(Path &path, int connectLightID, VCMType vcmType){
		Ray &lightRay = path[connectLightID];
		Ray &eyeRay = path[connectLightID + 1];
		if(vcmType == VC && (lightRay.isDeltaDirection || eyeRay.isDeltaDirection))
			return false;
		if(vcmType == VM && eyeRay.isDeltaDirection)
			return false;

		const Ray &preLightRay = path[MAX(connectLightID - 1, 0)];
		const Ray &preEyeRay = path[MIN(connectLightID + 2, path.size() - 1)];

		lightRay.direction = eyeRay.origin - lightRay.origin;
		lightRay.direction.normalize();
		eyeRay.direction = -lightRay.direction;

		if(vcmType == VC){
			lightRay.directionProb = 1;
			lightRay.isDeltaDirection = false;
			lightRay.radiance = preLightRay.evalBSDF(lightRay);
			if(lightRay.contactObj && connectLightID > 0)
				lightRay.insideObj = lightRay.contactNormal().dot(lightRay.direction) > 0 ? 
				preLightRay.insideObj : lightRay.contactObj;
		}

		eyeRay.directionProb = 1;
		eyeRay.isDeltaDirection = false;
		eyeRay.radiance = preEyeRay.evalBSDF(eyeRay);
		if(eyeRay.contactObj && connectLightID < path.size() - 2)
			eyeRay.insideObj = eyeRay.contactNormal().dot(eyeRay.direction) > 0 ?
			preEyeRay.insideObj : eyeRay.contactObj;

		return true;
	}

	/* \brief: Link vertexs used for calculating path probility.

	Return Ray ***starts from Vi to Vj***
	*/
	Ray VCMTracer::linkVertex(const Path &path, int connectLightID, int startID, bool reverse) const{
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
	bool VCMTracer::pathCanNotConnect(const Path &path) const{
		for(int i = 0; i < path.size() - 1; i++){
			if(!path[i].isDeltaDirection && !path[i+1].isDeltaDirection)
				return false;
		}
		return true;
	}

}