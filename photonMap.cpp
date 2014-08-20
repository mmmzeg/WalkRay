#include "stdafx.h"
#include "photonMap.h"
#include "noself.h"
#include "renderEngine.h"
#include "anisotropicGlossyObj.h"
#include "homogeneousVolume.h"
#include "HeterogeneousVolume.h"
#include <limits>

#define MERGE_LEN 10
#define MAX_RATIO 30.0

//#define NO_RAY_MARCHING
#define PPM

//#define ANI_PM
//#define RESEARCH


namespace LFJ{
	std::vector<vec3f> PhotonMap::renderPixels(){
		Camera &camera = getCamera();
		uint width = camera.mResolution.x, height = camera.mResolution.y;
		std::vector<vec3f> pixelColors(width * height, vec3f(0,0,0));
		
		omp_init_lock(&surfaceHashGridLock);
		omp_init_lock(&volumeHashGridLock);
		omp_init_lock(&debugPrintLock);

#ifdef RESEARCH
		std::vector<int> pixelMaps(pixelColors.size(), 0);
#endif

		prepareForLightSampling();
		
		mRadius = mBaseRadius;


		clock_t startTime = clock();


		for(uint s = 0; s < spp; s++){
			std::cout << "iteration : " << s << std::endl;
		
			engine->scene.updateSceneForMotionBlur();

			std::vector<vec3f> oneIterColors(pixelColors.size(), vec3f(0,0,0));
#ifdef PPM
			mRadius = MAX(mBaseRadius * sqrt(powf(s+1, mAlpha-1)), EPSILON);
#endif

			std::vector<Path*> pixelLightPaths(pixelColors.size(), NULL);
			std::vector<LightPoint> surfaceLightVertices(pixelColors.size());
			std::vector<LightPoint> volumeLightVertices(pixelColors.size());

			surfaceHashGrid.Reserve(pixelColors.size());
			volumeHashGrid.Reserve(pixelColors.size());

#pragma omp parallel for
			// step1: sample light paths and build range search struct independently for surface and volume
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
					lightPoint.photonType = lightPath[i].photonType;
					if(lightPoint.photonType == Ray::OUTVOL){
						omp_set_lock(&surfaceHashGridLock);
						surfaceLightVertices.push_back(lightPoint);
						omp_unset_lock(&surfaceHashGridLock);
					}
					if(lightPoint.photonType == Ray::INVOL){
						omp_set_lock(&volumeHashGridLock);
						volumeLightVertices.push_back(lightPoint);
						omp_unset_lock(&volumeHashGridLock);
					}
				}
			}
			std::cout<< "vol vertices= " << volumeLightVertices.size() << " sur vertices= " << surfaceLightVertices.size() << std::endl;
			
			surfaceHashGrid.Build(surfaceLightVertices, mRadius);
			volumeHashGrid.Build(volumeLightVertices, mRadius);

			// step2: calculate pixel colors by progressive photon mapping
#pragma omp parallel for
			for(int p = 0; p < pixelColors.size(); p++){
				Path eyePath;
#ifndef NO_RAY_MARCHING
				sampleMergePath(eyePath, camera.generateRay(p), 0);
#else
				samplePath(eyePath, camera.generateRay(p));
#endif

#ifdef RESEARCH
				if(eyePath[1].contactObj && eyePath[1].contactObj->anisotropic()){
					pixelMaps[p] = 1;
				}
#endif
				throughputByDensityEstimation(oneIterColors[p], eyePath, surfaceLightVertices, volumeLightVertices);
			}

#ifdef RESEARCH
			std::ofstream fout(engine->renderer->name + engine->scene.name+"pixelMap.txt");
			for(int p = 0; p < pixelMaps.size(); p++)
				fout << pixelMaps[p] << ' ' ;
			fout << std::endl;
			fout.close();
#endif
#pragma omp parallel for
			for(uint i = 0; i < pixelColors.size(); i++){
				pixelColors[i] *= s / float(s+1);
				if(!isLegalColor(oneIterColors[i]))
					oneIterColors[i] = vec3f(0,0,0);
				pixelColors[i] += camera.fixVignetting(oneIterColors[i], i) / (s + 1);
				delete pixelLightPaths[i];
			}
			camera.mFilm.setBuffer(pixelColors);
			float time = (float)(clock() - startTime) / 1000;
#ifdef RESEARCH

	#ifdef ANI_PM
/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
				std::string filename = engine->renderer->name + engine->scene.name + "_m_0.2_" + std::to_string(s) + ".bmp";
	#else 
				std::string filename = engine->renderer->name + engine->scene.name + "_s_trueStd_" + std::to_string(s) + ".bmp";
	#endif

#else
			std::string filename = engine->renderer->name + engine->scene.name + ".pfm";
#endif
			//camera.mFilm.saveBMP(filename);
			camera.mFilm.savePFM(filename);
		}
		return pixelColors;
	}

	void PhotonMap::sampleMergePath(Path &path, Ray &prevRay, uint depth) const{
		path.push_back(prevRay);
		Ray terminateRay;
		terminateRay.origin = prevRay.origin;
		terminateRay.radiance = vec3f(0,0,0);
		terminateRay.direction = vec3f(0,0,0);
		terminateRay.isDeltaDirection = true;
		terminateRay.insideObj = NULL;
		terminateRay.contactObj = NULL;
		terminateRay.intersectObj = NULL;

		Ray nextRay;
		if(prevRay.insideObj && !prevRay.insideObj->isVolume())			
			nextRay = prevRay.insideObj->scatter(prevRay);
		else if(prevRay.intersectObj){
			if(prevRay.intersectObj->isVolume() && prevRay.contactObj && prevRay.contactObj->isVolume()){
				prevRay.origin += prevRay.direction * prevRay.intersectDist;
				prevRay.intersectDist = 0;
			}
			nextRay = prevRay.intersectObj->scatter(prevRay);
		}
		else{
			path.push_back(terminateRay);	return ;
		}

		if(nextRay.direction.length() < 0.5){
			path.push_back(nextRay);		return ;
		}
		if(depth + 1 > MERGE_LEN){
			path.push_back(terminateRay);	return ;
		}
		NoSelfCondition condition(&engine->scene, nextRay);
		Scene::ObjSourceInfo info;
		float dist = engine->scene.intersect(nextRay, info, &condition);
		if(dist < 0){
			path.push_back(nextRay);
			path.push_back(terminateRay);
			return ;
		}
		else{
			nextRay.intersectObj = engine->scene.objects[info.objID];
			nextRay.intersectTriangleID = info.triID;
			nextRay.intersectDist = dist;
		}
		sampleMergePath(path, nextRay, depth + 1);
	}

	void PhotonMap::throughputByDensityEstimation(vec3f &color, Path &eyeMergePath, 
			std::vector<LightPoint> &surfaceVertices, std::vector<LightPoint> &volumeVertices, bool draw)
	{
		class Query{
			PhotonMap  *photonMap;
			vec3f	    contrib;
			vec3f		position;
			vec3f		hitNormal;
			float	    radius;
			int			photonsNum;
			Ray         outRay;
			float GaussianKernel(float mahalanobisDist) const{
				double exponent = exp((double)-mahalanobisDist/2);
				return exponent / (2*PI);
			}
			float Kernel(float distSqr, float radiusSqr) const{
				float s = MAX(0, 1 - distSqr / radiusSqr);
				return 3 * s * s / PI;
			}
		public:
			Query(PhotonMap *map, float r, int n) : photonMap(map), radius(r), photonsNum(n), THEdraw(false)
			{}
			bool volumeMedia;
			void SetContrib(const vec3f &color) { contrib = color; }
			void SetPosition(const vec3f &pos)  { position = pos; }
			void SetOutRay(const Ray &ray) { outRay = ray; }
			void SetNormal(const vec3f &n) { hitNormal = n; }
			vec3f GetContrib() const  { return contrib; }
			vec3f GetPosition() const { return position; }
			void Process(const LightPoint &lightPoint){
				if(volumeMedia && lightPoint.photonType != Ray::INVOL)		return ;
				if(!volumeMedia && lightPoint.photonType != Ray::OUTVOL)	return ;	
				if(!lightPoint.pathThePointIn || lightPoint.indexInThePath < 0)
					return ;

				Path &lightPath = *lightPoint.pathThePointIn;
				int index = lightPoint.indexInThePath;
				vec3f photonThroughput(1,1,1);
				for(int i = 0; i < index; i++){
					photonThroughput *= lightPath[i].radiance / lightPath[i].directionProb / lightPath[i].originProb;
					photonThroughput *= lightPath[i].cosineTerm();
					float dist = (lightPath[i].origin-lightPath[i+1].origin).length();
					photonThroughput *= lightPath[i].radianceDecay(dist);
				}
				photonThroughput /= lightPath[index].originProb;
				// runs here, photon's f/p is done.

				Ray photonRay = lightPath[index];
				photonRay.direction = lightPath[index-1].direction;
				vec3f color = photonThroughput * photonRay.evalBSDF(outRay);
				float distSqr = powf((outRay.origin-lightPath[index].origin).length(), 2);
				if(!isLegalColor(color))	return ;
#ifndef RESEARCH
				float kernel = Kernel(distSqr, radius*radius);
				float normalization = volumeMedia ? kernel/(photonsNum*radius*radius*radius) : kernel/(photonsNum*radius*radius);
#else
				float normalization = volumeMedia==false ? 1.0 / (photonsNum*PI*radius*radius) : 1.0 / (photonsNum*PI*4.0/3*radius*radius*radius);
#endif
				contrib += color * normalization;
			}

			double sumWeight; 
			int    photonsCount;
			bool   anisotropic;
			bool   THEdraw;
			void EllipseProcess(const LightPoint &lightPoint, const Frame *ellipseFrame, const vec3f &hitPoint, float aniRatio){
				if(volumeMedia && lightPoint.photonType != Ray::INVOL)	{return ;}
				if(!volumeMedia && lightPoint.photonType != Ray::OUTVOL){return ;}
				Path &lightPath = *lightPoint.pathThePointIn;
				int index = lightPoint.indexInThePath;
				vec3f photonThroughput(1,1,1);
				for(int i = 0; i < index; i++){
					photonThroughput *= lightPath[i].radiance / lightPath[i].directionProb / lightPath[i].originProb;
					photonThroughput *= lightPath[i].cosineTerm();
					float dist = (lightPath[i].origin-lightPath[i+1].origin).length();
					photonThroughput *= lightPath[i].radianceDecay(dist);
				}
				photonThroughput /= lightPath[index].originProb;
				// runs here, photon's f/p is done.

				Ray photonRay = lightPath[index];
				photonRay.direction = lightPath[index-1].direction;
				vec3f color = photonThroughput * photonRay.evalBSDF(outRay);
				float distSqr = powf((outRay.origin-lightPath[index].origin).length(), 2);
				if(!isLegalColor(color))	{return ;}
				
				// Multivariate Kernel Density Estimation.
				const vec3f &photonPos = lightPoint.position;
				const vec3f hitToPhoton = photonPos - hitPoint;
				const float X = hitToPhoton.dot(ellipseFrame->mX), Y = hitToPhoton.dot(ellipseFrame->mY);			
				const float 
					A = radius * aniRatio, B = radius;
					/*A = anisotropic ? radius * sqrt(aniRatio) : radius, 
					B = anisotropic ? radius / sqrt(aniRatio) : radius;*/

				float MaharashtraDist = X*X/(A*A) + Y*Y/(B*B);
				float weight = GaussianKernel(MaharashtraDist);
				sumWeight += weight;	photonsCount ++;
				float convergence = 1.0 / (photonsNum * PI * radius * radius);  
				contrib += weight * color * convergence;
			 
			}

			void weightScale(){
				contrib /= ( sumWeight / photonsCount );
			}

		};

		Query query(this, mRadius, mPhotonsNum);
		vec3f Tr(1,1,1), SurfaceColor(0,0,0), VolumeColor(0,0,0);
		int mergeIndex = 1;
		for(int i = 1; i < eyeMergePath.size(); i++){
			float dist = MAX((eyeMergePath[i-1].origin-eyeMergePath[i].origin).length(), EPSILON);
			if(eyeMergePath[i-1].insideObj && eyeMergePath[i-1].insideObj->isVolume()){
				if(eyeMergePath[i-1].insideObj->homogeneous()){
					// ray marching volume radiance
					Ray volThroughRay = eyeMergePath[i-1];
					HomogeneousVolume *volume = static_cast<HomogeneousVolume*>(volThroughRay.insideObj);
					float stepSize = volume->getStepSize();
					int N = dist / stepSize;
					if(N == 0)		N++;
					float step = dist / N;
					float offset = step * engine->rng->genFloat();
					float t = offset;
					Tr *= volume->randianceDecay(volThroughRay, offset);
					for(int j = 0; j < N; j++, t+=step){
						query.SetContrib(vec3f(0,0,0));
						query.SetPosition(volThroughRay.origin + volThroughRay.direction*t);
						Ray outRay = volThroughRay;
						outRay.direction = -volThroughRay.direction;
						outRay.origin = volThroughRay.origin + volThroughRay.direction*t;
						outRay.contactObj = NULL;
						query.SetOutRay(outRay);
						query.volumeMedia = true;
						volumeHashGrid.Process(volumeVertices, query);
						Tr *= volume->randianceDecay(outRay, step);
						vec3f volColor = query.GetContrib();
						VolumeColor += volColor * Tr * step;
					}
				}
				else{
					// ray marching volume radiance
					Ray volThroughRay = eyeMergePath[i-1];
					HeterogeneousVolume *volume = static_cast<HeterogeneousVolume*>(volThroughRay.insideObj);
					float stepSize = volume->getStepSize();
					int N = dist / stepSize;
					if(N == 0)		N++;
					float step = dist / N;
					float offset = step * engine->rng->genFloat();
					float t = offset;
					Tr *= volume->randianceDecay(volThroughRay, offset);
					for(int j = 0; j < N; j++, t+=step){
						query.SetContrib(vec3f(0,0,0));
						query.SetPosition(volThroughRay.origin + volThroughRay.direction*t);
						Ray outRay = volThroughRay;
						outRay.direction = -volThroughRay.direction;
						outRay.origin = volThroughRay.origin + volThroughRay.direction*t;
						outRay.contactObj = NULL;
						query.SetOutRay(outRay);
						query.volumeMedia = true;
						volumeHashGrid.Process(volumeVertices, query);
						Tr *= volume->randianceDecay(outRay, step);
						vec3f volColor = query.GetContrib();
						VolumeColor += volColor * Tr * step;
					}
				}
			}

			if(eyeMergePath[i].contactObj && eyeMergePath[i].contactObj->isEmissive()){
				// eye path hit light, surface color equals to light radiance
				SurfaceColor = eyeMergePath[i].radiance;
				mergeIndex = i;
				break;
			}

			if(eyeMergePath[i].contactObj && eyeMergePath[i].contactObj->nonSpecular()){
				// non-specular photon density estimation
				if(eyeMergePath[i].contactObj->isVolume())
					continue;
				query.SetContrib(vec3f(0,0,0));
				query.SetPosition(eyeMergePath[i].origin);
				Ray outRay = eyeMergePath[i];
				outRay.direction = -eyeMergePath[i-1].direction;
				query.SetOutRay(outRay);
				query.volumeMedia = false;
				Ray fromRay = eyeMergePath[i-1];
				if(fromRay.contactObj && fromRay.contactObj->anisotropic()){
					AnisotropicGlossyObject *aniObj = static_cast<AnisotropicGlossyObject*>(fromRay.contactObj);
					double Lambda = aniObj->Nu / 2, Miu = aniObj->Nv / 2; 
/**************************************************** process ellipse MERGE BEGIN*************************************************************************/
					/* start warping */
					Ray prevFromRay = eyeMergePath[i-2];
					const vec3f normal = fromRay.contactObj->getWorldNormal(fromRay.contactTriangleID, fromRay.origin);
					const vec3f &Z_h = normal, O = -prevFromRay.direction;
					vec3f I = 2*O.dot(Z_h)*Z_h - O;	 I.normalize();
					/* [step1: get brdf local frame, which was used when doing importance sampling.]*/				/* [Zh Xh Yh] */
					Frame BSDF_sample_frame;	BSDF_sample_frame.setFromN(normal);
					/* [step2: set local frame2 from Z_h, with tangent direction x_h' perpendicular to o.] *./		/* [Zh Xh' Yh'] */
					vec3f X_h_ = O.cross(Z_h);	X_h_.normalize();
					/* [step3: get X_i' and Y_i', which are used for transforming X_i and Y_i.] */					/* [Zi Xi' Yi'] */
					vec3f X_i_, Y_i_;	X_i_ = X_h_;	Y_i_ = I.cross(X_i_); 
					/* [step4: calculate Hessian Matrix.] */
					double HA, HB, HD, dotXX = (BSDF_sample_frame.mZ.dot(X_h_)), dotYX = (BSDF_sample_frame.mX.dot(X_h_));
					HA = Lambda * pow(dotXX, 2) + Miu * pow(dotYX, 2);		HA /= 4 * pow(O.dot(Z_h), 2);
					HB = (Miu - Lambda) * (dotXX * dotYX);					HB /= 4 * O.dot(Z_h);
					HD = Miu * pow(dotXX, 2) + Lambda * pow(dotYX, 2);		HD /= 4;
					/* [step5: eigen-decomposition Hessian Matrix and get transform Matrix U.] */
					double Lambda_I, Miu_I, cs, sn;
					dlaev2_(&HA, &HB, &HD, &Lambda_I, &Miu_I, &cs, &sn);
					Lambda_I = fabs(Lambda_I), Miu_I = fabs(Miu_I);
					if(fabs(cs) > 0.99 || fabs(sn) > 0.99){ // avoid extremely sharp ellipse
						surfaceHashGrid.Process(surfaceVertices, query);
						SurfaceColor = query.GetContrib();
						mergeIndex = i;		break;
					}
					
					/* [step6: apply the Matrix U's transform to obtain the outRay's BRDF local frame.] */
					Frame IFrame;	IFrame.mZ = I;
					IFrame.mX = cs * X_i_ + sn * Y_i_;
					IFrame.mY = -sn * X_i_ + cs * Y_i_;
					/////////////////////////////////////////////////////////////////////////
					// EVALUATE WARPING CODE : HERE( code is at the bottom of this file )
					// evaluate warping 
					//{   // Lambda > Miu, Lambda - IFrame.mX, Miu - IFrame.mY, uu - mX, vv - mY.
					//	// so when |uu| > |vv| with the same radius, the brdf should change more. 
					//	// so we assume maxAngle = 90 or 270 since that's where |uu| reaches max.
					//	
					//	omp_set_lock(&debugPrintLock);
					//	fout << std::endl;
					//	{
					//		Ray newFromRay = fromRay;
					//		double  uu = 0, vv = 0;
					//		newFromRay.direction = IFrame.mZ + IFrame.mX * uu + IFrame.mY * vv;
					//		newFromRay.direction.normalize();
					//		const vec3f cenBSDF = prevFromRay.evalBSDF(newFromRay);
					//		fout << "cenBSDF = " << cenBSDF << std::endl;
					//		double maxChange=0, maxAngle=0;
					//		for(int k = 0; k < 400 ; k++){
					//			double angle = k * PI / 200;
					//			uu = 0.005*sin(angle), vv = 0.005*cos(angle);
					//			newFromRay.direction = IFrame.mZ + IFrame.mX * uu + IFrame.mY * vv;
					//			newFromRay.direction.normalize();
					//			fout << "angle = " << 18.0/20*k << " u = " << uu << " v = " << vv << " bsdf = " << prevFromRay.evalBSDF(newFromRay) << "CAHGNE = " << (prevFromRay.evalBSDF(newFromRay) - cenBSDF).length() << std::endl;
					//			if((prevFromRay.evalBSDF(newFromRay) - cenBSDF).length() > maxChange){
					//				maxChange = (prevFromRay.evalBSDF(newFromRay) - cenBSDF).length();
					//				maxAngle = 18.0/20*k; 
					//			}
					//		} // assume maxAngle = 90 or 270
					//		double delta = MIN(abs(90-maxAngle), abs(270-maxAngle));
					//		fout << " lamdaI = " << Lambda_I << " u = " <<Miu_I  << " cs sn = " << cs << ' ' << sn << " maxAngle = " << maxAngle << " delta(90/270) = " << delta << std::endl; ;
					//	}
					//	omp_unset_lock(&debugPrintLock);
					//}	
					/////////////////////////////////////////////////////////////////////////
					/* warping is done. */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					const vec3f &sp_dir = fromRay.direction;
					const Frame &warping_frame = IFrame;
					// get a0, b0 and c0, which is the new ellipse's center mZ.
					const float a0 = sp_dir.dot(warping_frame.mX);
					const float b0 = sp_dir.dot(warping_frame.mY);
					const float c0 = sqrt(CLAMP(1 - a0*a0 - b0*b0, 0, 1));
					// calculate spherical coordinate theta and phi. 
					const double theta = acos(CLAMP(c0,-1,1)), sinTheta = sin(theta);
					const double absPhi = acos(CLAMP(a0 / sinTheta, -1, 1));
					const double phi = b0 > 0 ? absPhi : 2*PI - absPhi;
					// gen spherical coordinate vectors.
					Frame sphericalFrame;
					sphericalFrame.mZ = sp_dir; // r
					sphericalFrame.mY = warping_frame.mZ.cross(sp_dir); // phi
					sphericalFrame.mY.normalize();
					sphericalFrame.mX = sphericalFrame.mY.cross(sphericalFrame.mZ); // theta
					// calculate first and second derivatives.	
					const double delta = 0.0001;
					const double dF_dTheta = ASG_dF_dTheta(theta, phi, Lambda_I, Miu_I);
					const double dF_dPhi = ASG_dF_dPhi(theta, phi, Lambda_I, Miu_I);
					const double dF2_dTheta2 = (ASG_dF_dTheta(theta+delta, phi, Lambda_I, Miu_I) - ASG_dF_dTheta(theta-delta, phi, Lambda_I, Miu_I)) / (2*delta);
					const double dF2_dPhi2 = (ASG_dF_dPhi(theta, phi+delta, Lambda_I, Miu_I) - ASG_dF_dPhi(theta, phi-delta, Lambda_I, Miu_I)) / (2*delta);
					const double dF2_dTheta_dPhi = (ASG_dF_dTheta(theta, phi+delta, Lambda_I, Miu_I) - ASG_dF_dTheta(theta, phi-delta, Lambda_I, Miu_I)) / (2*delta);

					// obtain fastest and slowest direction.
					vec3f fastest = dF_dTheta * sphericalFrame.mX + dF_dPhi / sinTheta * sphericalFrame.mY;		
					fastest.normalize();
					vec3f slowest = sp_dir.cross(fastest);		
					
					// calculate alpha1 and alpha2.
					vec3f lenThetaPhi(dF_dTheta, dF_dPhi / sinTheta, 0); // length(theta, phi)
					lenThetaPhi.normalize();
					const double alpha1 = acos(fabs(lenThetaPhi.x)), alpha2 = PI/2 - alpha1;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
					const double err_bound = 0.002;
					const double lowerTerm = cos(theta) * exp(-sinTheta*sinTheta*(Lambda_I*cos(phi)*cos(phi)+Miu_I*sin(phi)*sin(phi)));
					const double Xi_f = fabs(err_bound*lowerTerm / (dF_dTheta*cos(alpha1) + dF_dPhi * sin(alpha1)/sinTheta)),
								 Xi_s = sqrt(fabs(2*err_bound*lowerTerm / (dF2_dTheta2*cos(alpha2)*cos(alpha2) + 2*dF2_dTheta_dPhi*sin(alpha2)*cos(alpha2)/sinTheta + dF2_dPhi2*sin(alpha2)*sin(alpha2)/(sinTheta*sinTheta))));
					
					
					
/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
					// 4. Control E_ratio by Un-projected Xi
					// A
					// const float ellipse_ratio = 1.0 + log(MAX(fabs(Xi_s/Xi_f),1.f));
					// B
					// const float ellipse_ratio = MAX(fabs(Xi_s/Xi_f), 1.f);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					// set up hitFrame.
					const vec3f hitNormal = outRay.contactObj->getWorldNormal(outRay.contactTriangleID, outRay.origin);
					Frame hitFrame;		hitFrame.setFromN(hitNormal);
					// do parallel projection.
					if(fabs(sp_dir.dot(warping_frame.mZ)) < EPSILON || fabs(sp_dir.dot(hitNormal)) < EPSILON || fabs(Xi_s / Xi_f) <= 1.0){
						surfaceHashGrid.Process(surfaceVertices, query);
						SurfaceColor = query.GetContrib();
						mergeIndex = i;
						break;
					}
					double matrixA[2][2] = {};
					const vec3f D = sp_dir;
					const vec3f Np = hitFrame.mY; 
					const double D_dot_Np = D.dot(Np);
					double middleTermMatrix[3][3] = {};
					double I3[3][3] = { {1,0,0}, {0,1,0}, {0,0,1} };
					for(int row = 0; row < 3; row++){
						for(int col = 0; col < 3; col++){
							middleTermMatrix[row][col] = I3[row][col] - D[row] * Np[col] / D_dot_Np;
						}
					}
					double JpT[2][3] = { {hitFrame.mZ[0], hitFrame.mZ[1], hitFrame.mZ[2]}, 
										 {hitFrame.mX[0], hitFrame.mX[1], hitFrame.mX[2]} };
					double prevTwoTermMatrix[2][3] = {};
					for(int row = 0; row < 2; row++){
						for(int col = 0; col < 3; col++){
							for(int k = 0; k < 3; k++){
								prevTwoTermMatrix[row][col] += JpT[row][k] * middleTermMatrix[k][col];
							}
						}
					}
					double Je[3][2] = { { fastest[0], slowest[0] },
								        { fastest[1], slowest[1] },
										{ fastest[2], slowest[2] } };
					for(int row = 0; row < 2; row++){
						for(int col = 0; col < 2; col++){
							for(int k = 0; k < 3; k++){
								matrixA[row][col] += prevTwoTermMatrix[row][k] * Je[k][col];
							}
						}
					}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
/////////////////////////////////////////////////////////////////////////////////////////////////////////////-->
					vec3f fast = Xi_f * (matrixA[0][0] * hitFrame.mZ + matrixA[1][0] * hitFrame.mX);
					vec3f slow = Xi_s * (matrixA[0][1] * hitFrame.mZ + matrixA[1][1] * hitFrame.mX);
					// vec3f fastBackup = fast, slowBackup = slow;
					// 3. Control E_ratio by projected Xi
					// A
					//const float ellipse_ratio = 1.0 + log(MAX(slow.length() / fast.length(), 1.0));
					// B
					const float ellipse_ratio = MAX(slow.length() / fast.length(), 1.0);
					fast.normalize();	slow.normalize();



					//// 1. Control E_ratio by BRDF
					//Ray newFromRay = fromRay;
					//float cenBSDF = Luminance(prevFromRay.evalBSDF(newFromRay));
					//vec3f point = outRay.origin + fast * mRadius, point2 = outRay.origin - fast * mRadius;
					//newFromRay.direction = point - fromRay.origin;		newFromRay.direction.normalize();
					//float fastBSDF1 = Luminance(prevFromRay.evalBSDF(newFromRay));
					//newFromRay.direction = point2 - fromRay.origin;		newFromRay.direction.normalize();
					//float fastBSDF2 = Luminance(prevFromRay.evalBSDF(newFromRay));
					//float fastBSDF = (fastBSDF1 + fastBSDF2) / 2.0;
					//point = outRay.origin + slow * mRadius, point2 = outRay.origin - slow * mRadius;
					//newFromRay.direction = point - fromRay.origin;		newFromRay.direction.normalize();
					//float slowBSDF1 = Luminance(prevFromRay.evalBSDF(newFromRay));
					//newFromRay.direction = point2 - fromRay.origin;		newFromRay.direction.normalize();
					//float slowBSDF2 = Luminance(prevFromRay.evalBSDF(newFromRay));
					//float slowBSDF = (slowBSDF1 + slowBSDF2) / 2.0;

					//// 2. Control E_ratio by ASG
					//vec3f spDir = sp_dir;
					//float cenBSDF = eval_ASG(warping_frame, spDir, Lambda_I, Miu_I);
					//vec3f point = outRay.origin + fast * mRadius, point2 = outRay.origin - fast * mRadius;
					//spDir = point - fromRay.origin; spDir.normalize();
					//float fastBSDF1 = eval_ASG(warping_frame, spDir, Lambda_I, Miu_I);
					//spDir = point2 - fromRay.origin; spDir.normalize();
					//float fastBSDF2 = eval_ASG(warping_frame, spDir, Lambda_I, Miu_I);
					//float fastBSDF = (fastBSDF1 + fastBSDF2) / 2.0;
					//point = outRay.origin + slow*mRadius, point2 = outRay.origin - slow*mRadius;
					//spDir = point - fromRay.origin; spDir.normalize();
					//float slowBSDF1 = eval_ASG(warping_frame, spDir, Lambda_I, Miu_I);
					//spDir = point2 - fromRay.origin; spDir.normalize();
					//float slowBSDF2 = eval_ASG(warping_frame, spDir, Lambda_I, Miu_I);
					//float slowBSDF = (slowBSDF1+slowBSDF2) / 2.0;


					// 5. Control E_ratio by Warped ASG
					// const float ellipse_ratio = sqrt(Lambda_I / Miu_I);


					//////////    A
					//		const float ellipse_ratio = 1.0 + log(MAX(fabs(fastBSDF-cenBSDF)/fabs(slowBSDF-cenBSDF), 1.0));//
					
					//////////    B
					//		const float ellipse_ratio = MAX(fabs( (fastBSDF-cenBSDF) / (slowBSDF-cenBSDF) ), 1.0f);//


					////   X
					Frame ellipseFrame;
					ellipseFrame.mZ = hitNormal; // normal 
					ellipseFrame.mX = slow;		 // longAxis
					ellipseFrame.mY = ellipseFrame.mZ.cross(ellipseFrame.mX);
		
					////   Y
					//Frame ellipseFrame;
					//ellipseFrame.mZ = hitNormal; // normal 
					//ellipseFrame.mX = slow;		 // longAxis
					//ellipseFrame.mY = fast;		 // shortAxis
		
					//////   Z
					//Frame ellipseFrame;
					//ellipseFrame.mZ = hitNormal; // normal 
					//ellipseFrame.mY = fast;		 // shortAxis
					//ellipseFrame.mX = ellipseFrame.mZ.cross(ellipseFrame.mY);

#ifdef ANI_PM

					omp_set_lock(&surfaceHashGridLock);
					query.sumWeight = 0;	query.photonsCount = 0;
					query.anisotropic = true;
					surfaceHashGrid.mEllipseFramePtr = &ellipseFrame;
					surfaceHashGrid.mHitPoint = outRay.origin;
					surfaceHashGrid.mAniRatio = ellipse_ratio;
					surfaceHashGrid.Process(surfaceVertices, query, true);
					query.weightScale();
					omp_unset_lock(&surfaceHashGridLock);

#else
					/*omp_set_lock(&surfaceHashGridLock);
					surfaceHashGrid.Process(surfaceVertices, query);
					omp_unset_lock(&surfaceHashGridLock);*/
					omp_set_lock(&surfaceHashGridLock);
					query.sumWeight = 0;	query.photonsCount = 0;
					query.anisotropic = false;
					surfaceHashGrid.mEllipseFramePtr = &ellipseFrame;
					surfaceHashGrid.mHitPoint = outRay.origin;
					surfaceHashGrid.mAniRatio = ellipse_ratio;
					surfaceHashGrid.Process(surfaceVertices, query, true);
					query.weightScale();
					omp_unset_lock(&surfaceHashGridLock);

#endif
/**************************************************** process ellipse MERGE END*************************************************************************/
				}
				else{
					surfaceHashGrid.Process(surfaceVertices, query);
				}

				SurfaceColor = query.GetContrib();
				mergeIndex = i;
				break;
			}
		}
		color = Tr * SurfaceColor + VolumeColor;

#ifndef NO_RAY_MARCHING
		for(int i = 0; i < 1/*eyeMergePath.size()-1*/; i++){
			color *= eyeMergePath[i].cosineTerm() * eyeMergePath[i].radiance
				/ eyeMergePath[i].directionProb / eyeMergePath[i].originProb;
		}
#else
		for(int i = 0; i < mergeIndex; i++){
			color *= eyeMergePath[i].cosineTerm() * eyeMergePath[i].radiance
				/ eyeMergePath[i].directionProb / eyeMergePath[i].originProb;
		}
#endif

		return ;
	}

	double PhotonMap::ASG_dF_dTheta(const double theta, const double phi, const double Lambda, const double Miu) const{
		const double param1 = Lambda * cos(phi) * cos(phi) + Miu * sin(phi) * sin(phi);
		const double sinTheta = sin(theta);
		return exp(-sinTheta*sinTheta*param1) * sinTheta * (2*cos(theta)*cos(theta)*(-param1) - 1);
	}

	double PhotonMap::ASG_dF_dPhi(const double theta, const double phi, const double Lambda, const double Miu) const{
		const double param1 = Lambda * cos(phi) * cos(phi) + Miu * sin(phi) * sin(phi);
		const double sinTheta = sin(theta);
		return exp(-sinTheta*sinTheta*param1) * 2 * sinTheta * sinTheta * cos(theta) * sin(phi) * cos(phi) * (Lambda - Miu);
	}

	double PhotonMap::ASG_dF2_dTheta(const double theta, const double phi, const double Lambda, const double Miu) const{
		const double param1 = Lambda * cos(phi) * cos(phi) + Miu * sin(phi) * sin(phi);
		const double sinTheta = sin(theta), cosTheta = cos(theta);
		const double expTerm = exp(-sinTheta*sinTheta*param1);
		const double firstTerm = -cosTheta + 2 * cosTheta * (1 - 3 * sinTheta * sinTheta) * ( -param1);
		const double secondTerm = -2*sinTheta * cosTheta * param1 * (2 * sinTheta * cosTheta * cosTheta * (-param1) - sinTheta );
		return expTerm * (firstTerm + secondTerm);
	}

	double PhotonMap::eval_ASG(const Frame &warpFrame, const vec3f &sp_dir, const double Lambda, const double Miu) const{
		const double a = sp_dir.dot(warpFrame.mX);
		const double b = sp_dir.dot(warpFrame.mY);
		const double c = sqrt(1- a*a - b*b);
		double ASG = c * exp(-Lambda * a* a -Miu * b * b);
		return ASG;
	}

	double PhotonMap::genEllipseRatio(const double cen, const double fast, const double slow){
		//omp_set_lock(&debugPrintLock);
		double ratio = 1.0;
		//std::cout << " fabs(fast-cen) = " << fabs(fast-cen) << " fabs(slow-cen) = " << fabs(slow-cen) << std::endl;
		//omp_unset_lock(&debugPrintLock);
		if(fabs(cen) < EPSILON || fabs(fast) < EPSILON || fabs(slow) < EPSILON){
			return ratio;
		}
		const double deltaFast = fabs(cen - fast), deltaSlow = fabs(cen - slow);
		if(fabs(deltaFast) < EPSILON || fabs(deltaSlow) < EPSILON || deltaFast < deltaSlow){
			return ratio;
		}
		else{
			ratio = 1.0 + log(deltaFast/deltaSlow) / log(10);
		}
		return CLAMP(ratio, 1.0, MAX_RATIO);
	}

	void PhotonMap::draw_ASG_ratios(){
		const float Lambda = 1000, Miu = 100; // aniRatio = 5
		Frame warpFrame; 
		warpFrame.setFromN(vec3f(0,1,0));
		const int width = 512, height = 512;
		IplImage *img_asgValue = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
		IplImage *img_ratio = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
		Film imgRatio, gradientAngle;
		imgRatio.setup(vec2f(width,height));
		gradientAngle.setup(vec2f(width,height));
		std::ofstream fout2("debug2.txt");

		for(int x = 0; x < width; x++){

			for(int y = 0; y < height; y++){
				float a = (float)(y - height/2) / 256, b = (float)(x - width/2) / 256;
				if(a*a + b*b > 1){
					vec3f &bgr2 = ((vec3f*)(img_ratio->imageData))[y*width+x];
					bgr2 = vec3f(0,0,0); 
					continue;
				}
				float c = sqrt(CLAMP(1-a*a-b*b, 0, 1));

				vec3f sp_dir = vec3f(a,b,c);	sp_dir = warpFrame.toWorldFrame(sp_dir);
				double ASG_value = eval_ASG(warpFrame, sp_dir, Lambda, Miu);
				vec3f &bgr = ((vec3f*)(img_asgValue->imageData))[y*width+x];
				bgr = vec3f(254*(1-ASG_value) ,0, 254*ASG_value); 

			

					// calculate spherical coordinate theta and phi. 
					const double theta = acos(CLAMP(c,-1,1)), sinTheta = sin(theta);
					const double absPhi = acos(CLAMP(a / sinTheta, -1, 1));
					const double phi = b > 0 ? absPhi : 2*PI - absPhi;

					// gen spherical coordinate vectors.
					Frame sphericalFrame;
					sphericalFrame.mZ = sp_dir; // r
					sphericalFrame.mY = warpFrame.mZ.cross(sp_dir); // phi
					sphericalFrame.mY.normalize();
					sphericalFrame.mX = sphericalFrame.mY.cross(sphericalFrame.mZ); // theta

					const double L = 0.001;;
					// calculate first and second derivatives.	
					const double delta = 0.0001;
					const double dF_dTheta = ASG_dF_dTheta(theta, phi, Lambda, Miu);
					const double dF_dPhi = ASG_dF_dPhi(theta, phi, Lambda, Miu);
					//const double dF2_dTheta2 = (ASG_dF_dTheta(theta+delta, phi, Lambda, Miu) - ASG_dF_dTheta(theta-delta, phi, Lambda, Miu)) / (2*delta);
					//const double dF2_dPhi2 = (ASG_dF_dPhi(theta, phi+delta, Lambda, Miu) - ASG_dF_dPhi(theta, phi-delta, Lambda, Miu)) / (2*delta);
					//const double dF2_dTheta_dPhi = (ASG_dF_dTheta(theta, phi+delta, Lambda, Miu) - ASG_dF_dTheta(theta, phi-delta, Lambda, Miu)) / (2*delta);

					//vec2d theta1(dF_dTheta, dF_dPhi / sinTheta);		
					//const double len1 = sqrt(theta1.x*theta1.x + theta1.y*theta1.y);
					//theta1 /= len1;
					//vec2d theta2(dF_dPhi / sinTheta, -dF_dTheta);		
					//const double len2 = sqrt(theta2.x*theta2.x + theta2.y*theta2.y);
					//theta2 /= len2;
					//// 1. obtain the deltaF in gradient direction(fastest dir).
					//const double deltaF1 = dF_dTheta * L * theta1.x + dF_dPhi * L * theta1.y / sinTheta;
					//// 2. obtain the deltaF in directional derivative direction(slowest dir).
					//const double deltaF2 = 0.5 * L * L * (dF2_dTheta2*theta2.x*theta2.x + 2*dF2_dTheta_dPhi*theta2.x*theta2.y/sinTheta + dF2_dPhi2*theta2.y*theta2.y/(sinTheta*sinTheta));
					//double ellipse_ratio = fabs(deltaF1 / deltaF2);

					vec3f fastest = dF_dTheta * sphericalFrame.mX + dF_dPhi / sinTheta * sphericalFrame.mY;		
					fastest.normalize();
					vec3f slowest = sp_dir.cross(fastest);		



					double cenBSDF = eval_ASG(warpFrame, sp_dir, Lambda, Miu);
					vec3f  spDir = sp_dir + L * fastest;		spDir.normalize();
					double fastBSDF = eval_ASG(warpFrame, spDir, Lambda, Miu);
					spDir = sp_dir + L * slowest;				spDir.normalize();
					double slowBSDF = eval_ASG(warpFrame, spDir, Lambda, Miu);
					//fout2 << cenBSDF << ' ' << fastBSDF << ' '<< slowBSDF << std::endl;
					const double ellipse_ratio = fabs((fastBSDF-cenBSDF) / (slowBSDF-cenBSDF));
					//fout2 << "ratio = " << ellipse_ratio << std::endl;
					//fout2 << ellipse_ratio<< ' ' << deltaF1 << ' ' << deltaF2<< std::endl;
					if(b == 0){
						fout2 << "sinTheta = " << sinTheta << " a / sinTheta = " << a/sinTheta << " theta =  " << theta << " phi = " << phi << " df_dtheta= " << dF_dTheta << " df_dPhi= " << dF_dPhi << " fastest = " << fastest << " slowest = " << slowest << std::endl;
						
					}

				
					vec3f longAxis = warpFrame.toLocalFrame(slowest);
					longAxis = vec3f(longAxis.x, 0, longAxis.z);
					longAxis.normalize();


					imgRatio.setColor(x, y, vec3f(ellipse_ratio,ellipse_ratio,ellipse_ratio));
					gradientAngle.setColor(x, y,longAxis );
					/*vec3f &bgr2 = ((vec3f*)(img_ratio->imageData))[y*width+x];
					bgr2 = vec3f(254*(gaussian) ,0, 254*(1-gaussian)); 
*/

			}

		}
		cvSaveImage("ASG_v.jpg", img_asgValue);
		imgRatio.savePFM("ratio.pfm");
		gradientAngle.savePFM("gradAngle.pfm");
	}
}
