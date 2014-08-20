#include "stdafx.h"
#include "scene.h"
#include "HeterogeneousVolume.h"
#include "noself.h"
#include <fstream>
#include <limits>

#define NEWTON_BISECTION_EPS 1e-4

namespace LFJ{

	void HeterogeneousVolume::loadDensityMap(const std::string &filename){
		std::cout << "load density map " << std::endl;
		std::cout << "density filename= "  << filename << std::endl;
		std::ifstream fin(filename.c_str());
		std::string nX, nY, nZ;
		fin >> nX >> nY >> nZ;
		mBBox.nx = std::stoi(nX), mBBox.ny = std::stoi(nY), mBBox.nz = std::stoi(nZ);
		const int mapSize = mBBox.nx * mBBox.ny * mBBox.nz;

		getBoundingBox(mBBox.p0, mBBox.p1);
		/*mBBox.p0 = vec3f(this->transform * vec4f(this->minCoord, 1));
		mBBox.p1 = vec3f(this->transform * vec4f(this->maxCoord, 1));*/
		std::cout << "Grid BBox p0=" << mBBox.p0 << " p1=" << mBBox.p1 << std::endl;
		std::cout << "Grid World BBox = " << (vec3f(this->transform * vec4f(mBBox.p0, 1))) << " " << (vec3f(this->transform * vec4f(mBBox.p1, 1))) << std::endl;

		mat4f inverseTransform = inverse(this->transform);
		const vec3f localMin = vec3f(inverseTransform * vec4f((vec3f(this->transform * vec4f(mBBox.p0, 1))), 1));
		const vec3f localMax = vec3f(inverseTransform * vec4f((vec3f(this->transform * vec4f(mBBox.p1, 1))), 1));

		std::cout << "inverse: LocalMin = " << localMin << " LocalMax = " << localMax << std::endl;


		std::cout << "mapSize = " << mapSize << std::endl;
		mDensityMap = new float[mapSize];
		for(int i = 0; i < mapSize; i++)
			fin >> mDensityMap[i];
		std::cout << "load density map done. " << std::endl;
	}

	void HeterogeneousVolume::loadSubSurfaceVolumeData(const std::string &fileScattering, const std::string &fileAbsorption){
		std::cout << "load SubSurface volume data " << std::endl;
		std::cout << "subsurface scattering data filename= "  << fileScattering << std::endl;
		std::cout << "subsurface absorption data filename= "  << fileAbsorption << std::endl;
		std::ifstream finScattering(fileScattering.c_str()), finAbsorption(fileAbsorption.c_str());
		std::string nX, nY, nZ;
		finScattering >> nX >> nY >> nZ;
		mBBox.nx = std::stoi(nX), mBBox.ny = std::stoi(nY), mBBox.nz = std::stoi(nZ);
		const int mapSize = mBBox.nx * mBBox.ny * mBBox.nz;

		getBoundingBox(mBBox.p0, mBBox.p1);

		std::cout << "Grid BBox p0=" << mBBox.p0 << " p1=" << mBBox.p1 << std::endl;
		std::cout << "mapSize = " << mapSize << std::endl;
		mSubSurfaceDensityMap_Scattering = new vec3f[mapSize];
		mSubSurfaceDensityMap_Absorption = new vec3f[mapSize];
		mSubSurfaceDensityMap_Extinction = new vec3f[mapSize];

		vec3f scatteringVec3f, absorptionVec3f;
		for(int i = 0; i < mapSize; i++){
			finScattering >> scatteringVec3f.x >> scatteringVec3f.y >> scatteringVec3f.z;
			finAbsorption >> absorptionVec3f.x >> absorptionVec3f.y >> absorptionVec3f.z;
			mSubSurfaceDensityMap_Scattering[i] = scatteringVec3f * scatteringScale;
			mSubSurfaceDensityMap_Absorption[i] = absorptionVec3f * absorptionScale;
			mSubSurfaceDensityMap_Extinction[i] = mSubSurfaceDensityMap_Scattering[i] + mSubSurfaceDensityMap_Absorption[i];
		//	std::cout << "Volume ds = " << mSubSurfaceDensityMap_Scattering[i] << " da = " << mSubSurfaceDensityMap_Absorption[i] << " dt = " << mSubSurfaceDensityMap_Extinction[i] << std::endl;
		}
		std::cout << "load SubSurface volume data done. " << std::endl;
	}

	inline float HeterogeneousVolume::lookUpDensity(const vec3f &worldPos) const{
		if(!checkIn(worldPos, objID))
			return 0;
		mat4f inverseTransform = inverse(this->transform);
		const vec3f localPos = vec3f(inverseTransform * vec4f(worldPos, 1));
		if(localPos.x > mBBox.p1.x || localPos.x < mBBox.p0.x ||
			localPos.y > mBBox.p1.y || localPos.y < mBBox.p0.y ||
			localPos.z > mBBox.p1.z || localPos.z < mBBox.p0.z)
		{
			return 0;
		}

		float boundX = mBBox.p1.x - mBBox.p0.x,
			boundY = mBBox.p1.y - mBBox.p0.y,
			boundZ = mBBox.p1.z - mBBox.p0.z;

		float pToMinX = localPos.x - mBBox.p0.x,
			pToMinY = localPos.y - mBBox.p0.y,
			pToMinZ = localPos.z - mBBox.p0.z;

		int indexX = std::floor(mBBox.nx * pToMinX/boundX),
			indexY = std::floor(mBBox.ny * pToMinY/boundY),
			indexZ = std::floor(mBBox.nz * pToMinZ/boundZ);

		if(indexX < 0 || indexX >= mBBox.nx || 
			indexY < 0 || indexY >= mBBox.ny ||
			indexZ < 0 || indexZ >= mBBox.nz)
		{
			return 0;
		}

		float dx = mBBox.nx * pToMinX/boundX - indexX, 
			dy = mBBox.ny * pToMinY/boundY - indexY, 
			dz = mBBox.nz * pToMinZ/boundZ - indexZ;
		// Trilinearly interpolate density values to compute local density
		float sd00 = Lerp(dx, D(indexX, indexY, indexZ), D(indexX+1, indexY, indexZ));
		float sd10 = Lerp(dx, D(indexX, indexY+1, indexZ), D(indexX+1, indexY+1, indexZ));
		float sd01 = Lerp(dx, D(indexX, indexY, indexZ+1), D(indexX+1, indexY, indexZ+1));
		float sd11 = Lerp(dx, D(indexX, indexY+1, indexZ+1), D(indexX+1, indexY+1, indexZ+1));

		float sd0 = Lerp(dy, sd00, sd10);
		float sd1 = Lerp(dy, sd01, sd11);
		return Lerp(dz, sd0, sd1);

		return D(indexX, indexY, indexZ);
	}

	inline vec3f HeterogeneousVolume::lookUpSubSurfaceVolumeData(const vec3f &worldPos, LOOK_UP_TYPE type) const{
		if(!checkIn(worldPos, objID))
			return vec3f(0.f);
		mat4f inverseTransform = inverse(this->transform);
		const vec3f localPos = vec3f(inverseTransform * vec4f(worldPos, 1));
		if(localPos.x > mBBox.p1.x || localPos.x < mBBox.p0.x ||
			localPos.y > mBBox.p1.y || localPos.y < mBBox.p0.y ||
			localPos.z > mBBox.p1.z || localPos.z < mBBox.p0.z)
		{
			return vec3f(0.f);
		}

		float boundX = mBBox.p1.x - mBBox.p0.x,
			boundY = mBBox.p1.y - mBBox.p0.y,
			boundZ = mBBox.p1.z - mBBox.p0.z;

		float pToMinX = localPos.x - mBBox.p0.x,
			pToMinY = localPos.y - mBBox.p0.y,
			pToMinZ = localPos.z - mBBox.p0.z;

		int indexX = std::floor(mBBox.nx * pToMinX/boundX),
			indexY = std::floor(mBBox.ny * pToMinY/boundY),
			indexZ = std::floor(mBBox.nz * pToMinZ/boundZ);

		if(indexX < 0 || indexX >= mBBox.nx || 
			indexY < 0 || indexY >= mBBox.ny ||
			indexZ < 0 || indexZ >= mBBox.nz)
		{
			return vec3f(0.f);
		}

		float dx = mBBox.nx * pToMinX/boundX - indexX, 
			dy = mBBox.ny * pToMinY/boundY - indexY, 
			dz = mBBox.nz * pToMinZ/boundZ - indexZ;

		// Trilinearly interpolate density values to compute local density
		vec3f sd00 = Lerp(dx, SD(indexX, indexY, indexZ, type), SD(indexX+1, indexY, indexZ, type));
		vec3f sd10 = Lerp(dx, SD(indexX, indexY+1, indexZ, type), SD(indexX+1, indexY+1, indexZ, type));
		vec3f sd01 = Lerp(dx, SD(indexX, indexY, indexZ+1, type), SD(indexX+1, indexY, indexZ+1, type));
		vec3f sd11 = Lerp(dx, SD(indexX, indexY+1, indexZ+1, type), SD(indexX+1, indexY+1, indexZ+1, type));

		vec3f sd0 = Lerp(dy, sd00, sd10);
		vec3f sd1 = Lerp(dy, sd01, sd11);
		return Lerp(dz, sd0, sd1);
	}

	inline bool HeterogeneousVolume::checkIn(const vec3f &worldPos, const int objID) const{
		Ray ray;
		ray.origin = worldPos;
		ray.direction = vec3f(0,1,0);
		return this == scene->findInsideObjectWithID(ray, objID);
	}

	inline int HeterogeneousVolume::check(const Ray &inRay, float *intersectDist) const{
		bool contactIsVol = inRay.contactObj && inRay.contactObj == this;
		if(!checkIn(inRay.origin, objID) && !contactIsVol){
			return 1;
		}
		NoSelfCondition condition(scene, inRay);
		Scene::ObjSourceInfo info;
		float d = scene->intersect(inRay, info, &condition);
		if(!(d > 0)){
			return 2;
		}
		if(intersectDist)
			*intersectDist = d;
		return 0;
	}

	float HeterogeneousVolume::integrateDensity(const Ray &inRay, float dist) const{
		float densityAccumulation = 0;
		float intersectDist = std::numeric_limits<float>::max();
		if(check(inRay, &intersectDist)){
			return densityAccumulation;
		}
		dist = MIN(dist, intersectDist);

		vec3f p = inRay.origin;
		uint nSteps = std::ceil(dist / (2*stepSize));
		double ss = dist / nSteps, multiplier = (1.0/6.0)*ss;
		const vec3f fullStep = inRay.direction * ss, halfStep = fullStep * 0.5;

		float node1 = isSubsurface ? Luminance(lookUpSubSurfaceVolumeData(p, EXTINCTION)) : lookUpDensity(p);

		for(uint i = 0; i < nSteps; i++){
			float node2 = isSubsurface ? Luminance(lookUpSubSurfaceVolumeData(p+halfStep, EXTINCTION)) : lookUpDensity(p+halfStep), 
				node3 = isSubsurface ? Luminance(lookUpSubSurfaceVolumeData(p+fullStep, EXTINCTION)) : lookUpDensity(p+fullStep);
			densityAccumulation += multiplier*(node1+node2*4+node3);
			node1 = node3;
			p += fullStep;
		}
		return densityAccumulation;
	}

	vec3f HeterogeneousVolume::tau(const Ray &inRay, float dist) const{
		vec3f tauAccumulation = vec3f(0.f);
		float intersectDist = std::numeric_limits<float>::max();
		if(check(inRay, &intersectDist)){
			return tauAccumulation;
		}
		dist = MIN(dist, intersectDist);

		vec3f p = inRay.origin;
		uint nSteps = std::ceil(dist / (2*stepSize));
		double ss = dist / nSteps, multiplier = (1.0/6.0)*ss;
		const vec3f fullStep = inRay.direction * ss, halfStep = fullStep * 0.5;

		vec3f node1 = isSubsurface ? lookUpSubSurfaceVolumeData(p, EXTINCTION) : extinctionCoeff*lookUpDensity(p);

		for(uint i = 0; i < nSteps; i++){
			vec3f node2 = isSubsurface ? lookUpSubSurfaceVolumeData(p+halfStep, EXTINCTION) : extinctionCoeff*lookUpDensity(p+halfStep), 
				node3 = isSubsurface ? lookUpSubSurfaceVolumeData(p+fullStep, EXTINCTION) : extinctionCoeff*lookUpDensity(p+fullStep);
			tauAccumulation += multiplier*(node1+node2*4+node3);
			node1 = node3;
			p += fullStep;
		}
		return tauAccumulation;
	}

	float HeterogeneousVolume::pMedium(const Ray &inRay, float dist) const{
		const float tau = HeterogeneousVolume::integrateDensity(inRay, dist);
		const vec3f sigmaAtT = isSubsurface ?
			lookUpSubSurfaceVolumeData(inRay.origin + inRay.direction * dist, EXTINCTION): 
			extinctionCoeff*lookUpDensity(inRay.origin + inRay.direction * dist);
		return Luminance(sigmaAtT) * exp(-tau);
	}

	float HeterogeneousVolume::PSurface(const Ray &inRay, float dist) const{
		const float tau = HeterogeneousVolume::integrateDensity(inRay, dist);
		return exp(-tau);
	}

	float HeterogeneousVolume::getAlbedo() const{
		return Luminance(scatteringCoeff) / Luminance(extinctionCoeff);
	}

	float HeterogeneousVolume::getAlbedo(const vec3f &p) const{
		return Luminance(lookUpSubSurfaceVolumeData(p, SCATTERING)) / Luminance(lookUpSubSurfaceVolumeData(p, EXTINCTION));
	}

	int HeterogeneousVolume::findDesiredIntegralDensity(const Ray &inRay, const float desiredDensity, 
		float &t, float &integratedDensity, float &densityAtMinT, float &densityAtT) const
	{
		float dist;
		if(check(inRay, &dist)){
			return 1;
		}

		integratedDensity = 0;

		vec3f p = inRay.origin;
		uint nSteps = std::ceil(dist / (2*stepSize));
		double ss = dist / nSteps, multiplier = (1.0/6.0)*ss;
		const vec3f fullStep = inRay.direction * ss, halfStep = fullStep * 0.5;

		float node1 = isSubsurface ? Luminance(lookUpSubSurfaceVolumeData(p, EXTINCTION)) : lookUpDensity(p);
		densityAtMinT = node1;


		for(uint i = 0; i < nSteps; i++){
			float node2 = isSubsurface ? Luminance(lookUpSubSurfaceVolumeData(p+halfStep, EXTINCTION)) : lookUpDensity(p+halfStep), 
				node3 = isSubsurface ? Luminance(lookUpSubSurfaceVolumeData(p+fullStep, EXTINCTION)) : lookUpDensity(p+fullStep);
			float newDensity = integratedDensity + multiplier*(node1+node2*4+node3);
			if(newDensity >= desiredDensity){
				float a = 0, b = ss, x = a,
					fx = integratedDensity - desiredDensity,
					stepSizeSqr = ss * ss,
					temp = 1.0 / stepSizeSqr;

				int it = 1;
				while(true){
					float dfx = temp * (node1 * stepSizeSqr
						- (3 * node1 - 4 * node2 + node3) * ss * x
						+ 2 * (node1 - 2 * node2 + node3) * x * x);

					x -= fx / dfx;

					if(x <= a || x >= b || dfx == 0){
						x = 0.5 * (b + a);
					}

					float intval = integratedDensity + temp * (1.0/6.0) * (x *
						(6 * node1 * stepSizeSqr - 3 * (3 * node1 - 4 * node2 + node3) * ss * x
						+ 4 * (node1 - 2 * node2 + node3) * x * x));
					fx = intval - desiredDensity;

					if(std::abs(fx) < NEWTON_BISECTION_EPS){
						t = x + ss * i;
						integratedDensity = intval;
						densityAtT = temp * (node1 * stepSizeSqr
							- (3*node1 - 4*node2 + node3)*ss*x
							+ 2*(node1 - 2*node2 + node3)*x*x);
						return 0;
					}
					else if(++it > 30){
						// we still use the distance sample.
						t = x + ss * i;
						integratedDensity = intval;
						densityAtT = temp * (node1 * stepSizeSqr
							- (3*node1 - 4*node2 + node3)*ss*x
							+ 2*(node1 - 2*node2 + node3)*x*x);

				/*		std::cerr << "findDesiredIntegralDensity(): stuck in Newton-Bisection -- fx = " << fx << " dfx = " << dfx 
							<< " a = " << a << " b = " << b << " stepsize = " << ss << std::endl;*/
						return 2;
					}

					if(fx > 0){
						b = x;
					}
					else{
						a = x;
					}

				}
			}
			vec3f next = p + fullStep;
			if(p == next){
			//	std::cerr << "findDesiredIntegralDensity(): can not make forward progress -- stepsize = " << ss << std::endl;
				return 3;
			}
			integratedDensity = newDensity;
			node1 = node3;
			p = next;
		}
		return 4;
	}


	// 0 - success
	// 1 - check fail
	// 2 - iter > 30
	// 3 - can not make forward progress
	// 4 - finally failed. 
	// return true:		sample succeed, use pdfSuccess
	//		  false:	sample fail, use pdfFailure
	bool HeterogeneousVolume::sampleDistance(const Ray &inRay, float &distance, float &pdfSuccess, float &pdfFailure) const{
		float intersectDist;
		float desiredDensity = isSubsurface ? -log(1.0 - rng->genFloat()) : -log(1.0 - rng->genFloat()) / Luminance(extinctionCoeff);
		float t = 0, integratedDensity = 0, densityAtMinT = 0, densityAtT = 0;

		int flag = findDesiredIntegralDensity(inRay, desiredDensity, t, integratedDensity, densityAtMinT, densityAtT);
		float expVal = exp(-integratedDensity);

		bool sampleState = false;
		switch(flag){
		case 0:
			// success
			// satisfying: [desiredDensity = integratedDensity], [denisityAtT is real], [sample distance = t].
			pdfSuccess = expVal * densityAtT;
			distance = t;
			sampleState = true;
			break;
		case 1:
			// check fail
			// this one need extra calculation
			pdfFailure = expVal;
			distance = inRay.intersectDist;
			sampleState = false;
			break;

		case 2:
			// it > 30
			// we may use the integratedDensity with errorbound
			pdfSuccess = expVal * densityAtT;
			distance = t;
			sampleState = true;
			break;

		case 3:
			// stuck in progress
			pdfFailure = expVal;
			distance = inRay.intersectDist;
			sampleState = false;
			break;
		case 4:
			// finally failed.
			pdfFailure = expVal;
			distance = inRay.intersectDist;
			sampleState = false;
			break;
		default:
			break;
		}

		return sampleState;
	}

	vec3f HeterogeneousVolume::evaluateBSDF(const Ray &inRay, const Ray &outRay) const{
		if(!outRay.contactObj){
			if(!isSubsurface){
				vec3f BSDF = scatteringCoeff * bsdf->evalBSDF(inRay.direction, outRay.direction, vec3f(0,0,0));
				return BSDF * lookUpDensity(outRay.origin);
			}
			else{
				vec3f BSDF = bsdf->evalBSDF(inRay.direction, outRay.direction, vec3f(0,0,0));
				return BSDF * lookUpSubSurfaceVolumeData(outRay.origin, SCATTERING);
			}
		}
		if(outRay.contactObj && outRay.contactObj != this)
			return outRay.contactObj->evaluateBSDF(inRay, outRay);
		return vec3f(0,0,0);
	}

	float HeterogeneousVolume::evalOriginProbability(const Ray &inRay, const Ray &outRay) const{
		float dist = MAX((inRay.origin - outRay.origin).length(), EPSILON);

		return outRay.contactObj ?
			PSurface(inRay, dist):
			pMedium(inRay, dist);
	}

	float HeterogeneousVolume::evalDirectionProbability(const Ray &inRay, const Ray &outRay) const{
		if(outRay.isDeltaDirection)		return 0;
		if(outRay.contactObj)
			return outRay.contactObj->evalDirectionProbability(inRay, outRay);
		float continueAlbedo = isSubsurface ? getAlbedo(outRay.origin) : getAlbedo();
		float oPdfW = bsdf->evalPdfW(inRay.direction, outRay.direction, vec3f());
		return continueAlbedo * oPdfW;
	}

	vec3f HeterogeneousVolume::randianceDecay(const Ray &inRay, const float &dist) const{
		const vec3f tau = HeterogeneousVolume::tau(inRay, dist/*, true*/);
		vec3f Tr;
		for(int i = 0; i < 3; i++){
			Tr[i] = exp(-tau[i]);
		}
		return Tr;
	}


	Ray HeterogeneousVolume::scatter(Ray &inRay) const{
		Ray outRay;
		outRay.isDeltaDirection = false;

		bool go_in_vol = inRay.intersectObj == this && inRay.insideObj != this;
		bool be_in_vol = inRay.insideObj == this;

		// CASE1: Go in volume.
		if(go_in_vol){
			vec3f position = inRay.origin + inRay.direction*inRay.intersectDist;
			vec3f normal = inRay.intersectObj->getWorldNormal(inRay.intersectTriangleID, position);

			outRay.origin = position;
			outRay.direction = inRay.direction;

			vec3f reflDir = -normal.dot(inRay.direction)*normal*2 + inRay.direction;
			reflDir.normalize();
			float theta = acos(inRay.direction.dot(normal));

			AbstractObject* currentInsideObject = inRay.insideObj;
			AbstractObject* outSideObject = (AbstractObject*)this;

			float current_n = currentInsideObject ? currentInsideObject->getIOR() : 1;
			float next_n = outSideObject ? outSideObject->getIOR() : 1;
			float sin_phi = current_n / next_n * sin(theta);

			outRay.intersectObj = NULL;
			outRay.radiance = vec3f(1, 1, 1);
			outRay.directionProb = 1;
			outRay.contactObj = (AbstractObject*)this;
			outRay.contactTriangleID = inRay.intersectTriangleID;

			if(sin_phi > 1){
				outRay.direction = reflDir;
				outRay.insideObj = inRay.insideObj;
				outRay.directionProb = 1;
				outRay.isDeltaDirection = true;
				outRay.photonType = Ray::NOUSE;
			}
			else{
				float phi = asin(sin_phi);
				if(theta > PI/2)	phi = PI - phi;
				vec3f axis = normal.cross(inRay.direction);
				axis.normalize();
				outRay.direction = vec3f(RotateMatrix(axis, phi) * vec4f(normal, 0));
				outRay.direction.normalize();

				float cos_theta = abs(cos(theta));
				float cos_phi = abs(cos(phi));
				float esr = powf(abs(current_n*cos_theta-next_n*cos_phi)/(current_n*cos_theta+next_n*cos_phi),2);
				float epr = powf(abs(next_n*cos_theta-current_n*cos_phi)/(next_n*cos_theta+current_n*cos_phi),2);
				float er = (esr+epr)/2;
				float p = er;

				if(rng->genFloat() < p)
				{
					outRay.direction = reflDir;
					outRay.radiance *= er / outRay.cosineTerm();
					outRay.directionProb = p;
					outRay.insideObj = inRay.insideObj;
					outRay.isDeltaDirection = true;
					outRay.photonType = Ray::NOUSE;
				}
				else
				{
					outRay.radiance *= (1-er) / outRay.cosineTerm();
					outRay.directionProb = 1-p;
					outRay.contactObj = outRay.insideObj = (AbstractObject*)this;
					outRay.isDeltaDirection = true;
					outRay.photonType = Ray::HITVOL;
				}
				outRay.direction.normalize();
			}
			return outRay;
		}

		float p_medium, P_surface, sampleDist;

		bool samplingState = sampleDistance(inRay, sampleDist, p_medium, P_surface);

		bool out_of_vol = samplingState == false;//sampleDist >= inRay.intersectDist;

		// CASE2: Be in volume.
		if(be_in_vol && !out_of_vol){
			outRay.origin = inRay.origin + inRay.direction * sampleDist;
			outRay.radiance = bsdf->sampleBSDF(inRay.direction, outRay.direction, vec3f(), *rng, &outRay.directionProb);
			outRay.insideObj = (AbstractObject*)this;
			outRay.contactTriangleID = inRay.intersectTriangleID;
			float albedo = isSubsurface ? getAlbedo(outRay.origin) : getAlbedo();
			float rander = rng->genFloat();	
			if(rander < albedo){
				outRay.contactObj = NULL;
				outRay.directionProb *= albedo;
				outRay.originProb = p_medium;//  pMedium(inRay, sampleDist);// 
				outRay.isDeltaDirection = false;
				outRay.radiance *= isSubsurface ? lookUpSubSurfaceVolumeData(outRay.origin, SCATTERING) : scatteringCoeff * lookUpDensity(outRay.origin);
				outRay.photonType = Ray::INVOL;
			}
			else{
				// terminate
				outRay.direction = vec3f(0, 0, 0); 
				outRay.radiance = vec3f(0, 0, 0);  
				outRay.directionProb = 1; 
				outRay.originProb = p_medium;//pMedium(inRay, sampleDist);//
				outRay.insideObj = NULL;
				outRay.contactObj = NULL;
				outRay.isDeltaDirection = false;
				outRay.photonType = Ray::INVOL; //Ray::NOUSE;//
			}
			return outRay;
		}

		// CASE3: Go out of volume.
		if(be_in_vol && out_of_vol){
			outRay = inRay;
			outRay.direction = inRay.direction;
			outRay.origin = inRay.origin + inRay.intersectDist * inRay.direction;
			outRay.contactObj = inRay.intersectObj;
			outRay.contactTriangleID = inRay.intersectTriangleID;
			outRay.insideObj = (AbstractObject*)this;
			outRay.directionProb = 1; 
			outRay.radiance = vec3f(1,1,1);
			bool going_out = (inRay.intersectObj == this);
			if(going_out){
				vec3f normal = inRay.intersectObj->getWorldNormal(inRay.intersectTriangleID, outRay.origin);
				vec3f reflDir = -normal.dot(inRay.direction)*normal*2 + inRay.direction;
				reflDir.normalize();
				float theta = acos(inRay.direction.dot(normal));

				AbstractObject* currentInsideObject = (AbstractObject*)this;
				AbstractObject* outSideObject = scene->findInsideObject(outRay, (AbstractObject*)this);

				float current_n = currentInsideObject ? currentInsideObject->getIOR() : 1;
				float next_n = outSideObject ? outSideObject->getIOR() : 1;
				float sin_phi = current_n / next_n * sin(theta);

				outRay.intersectObj = NULL;
				if(sin_phi > 1){
					outRay.direction = reflDir;
					outRay.insideObj = inRay.insideObj;
					outRay.contactObj = (AbstractObject*)this;
					outRay.originProb = P_surface;// PSurface(inRay, inRay.intersectDist);//
					outRay.photonType = Ray::NOUSE;
					outRay.isDeltaDirection = true;
				}
				else{
					float phi = asin(sin_phi);
					if(theta > PI/2)	phi = PI - phi;
					vec3f axis = normal.cross(inRay.direction);
					axis.normalize();
					outRay.direction = vec3f(RotateMatrix(axis, phi) * vec4f(normal, 0));
					outRay.direction.normalize();

					float cos_theta = abs(cos(theta));
					float cos_phi = abs(cos(phi));
					float esr = powf(abs(current_n*cos_theta-next_n*cos_phi)/(current_n*cos_theta+next_n*cos_phi),2);
					float epr = powf(abs(next_n*cos_theta-current_n*cos_phi)/(next_n*cos_theta+current_n*cos_phi),2);
					float er = (esr+epr)/2;
					float p = er;

					if(rng->genFloat() < p)
					{
						outRay.direction = reflDir;
						outRay.radiance *= er / outRay.cosineTerm();
						outRay.directionProb = p;
						outRay.originProb = P_surface;// PSurface(inRay, inRay.intersectDist);//
						outRay.insideObj = inRay.insideObj;
						outRay.isDeltaDirection = true;
						outRay.photonType = Ray::NOUSE;
					}
					else
					{
						outRay.radiance *= (1-er) / outRay.cosineTerm();
						outRay.directionProb = (1-p);
						outRay.originProb = P_surface;//PSurface(inRay, inRay.intersectDist);// 
						outRay.insideObj = outSideObject;
						outRay.isDeltaDirection = true;
						outRay.photonType = Ray::NOUSE;
					}
					outRay.direction.normalize();
				}	
			}
			else{
				outRay.contactObj = NULL;
				outRay.intersectDist = 0;
				outRay = inRay.intersectObj->scatter(outRay);
				outRay.originProb *= P_surface;//PSurface(inRay, inRay.intersectDist);// 
				outRay.photonType = inRay.intersectObj->isVolume() ? Ray::NOUSE : Ray::OUTVOL;
			}
			return outRay;
		}
		return outRay;
	}


}