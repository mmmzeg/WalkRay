#include "abstractObj.h"

namespace LFJ{
	class EmissiveObject : public AbstractObject{
	private:
		vec3f color;
	public:
		EmissiveObject(Scene *scene) : AbstractObject(scene)
		{}

		bool isEmissive() const { return true; }
		void setColor(const vec3f &c) { color = c; }
		vec3f evaluateBSDF(const Ray &inRay, const Ray &outRay) const { return color; }
		vec3f getColor() const { return color; }
		Ray scatter(Ray &inRay) const{
			Ray outRay;
			vec3f position = inRay.origin + inRay.direction * inRay.intersectDist;
			vec3f worldNormal = inRay.intersectObj->getWorldNormal(inRay.intersectTriangleID, position);
			outRay.origin = position;
			outRay.direction = vec3f(0,0,0);
			outRay.intersectObj = NULL;
			outRay.insideObj = NULL;
			outRay.contactObj = (AbstractObject*)this;
			outRay.contactTriangleID = inRay.intersectTriangleID;
			
			outRay.radiance = inRay.direction.dot(worldNormal) <= 0 ? this->color : vec3f(0,0,0);
			return outRay;
		}
	};
}