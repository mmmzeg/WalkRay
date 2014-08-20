#pragma once
#include "macros.h"
#include "film.h"
#include "nvMatrix.h"

namespace LFJ{
	class TextureMap{
	protected:
		Film normalMap;
		Film diffuseMap;
		mat4f transform;
		mat4f inverseTransform;
		vec3f p0, p1;
	public:
		TextureMap(const mat4f t, vec3f p_min, vec3f p_max) : 
			transform(t), p0(p_min), p1(p_max)
		{
			inverseTransform = inverse(transform);
		} 
		void loadTextureMap(const std::string &diffMap, const std::string &normMap){
			loadDiffuseMap(diffMap);
			loadNormalMap(normMap);
		}
		void loadDiffuseMap(const std::string &diffMap){
			diffuseMap.loadImage(diffMap);
		}
		void loadNormalMap(const std::string &normMap){
			normalMap.loadImage(normMap);
		}

		vec3f toLocal(const vec3f &worldPos) const{
			vec3f localPos = vec3f(inverseTransform * vec4f(worldPos, 1));
			return localPos;
		}

		virtual vec2f getOffset(const vec3f &localPos) const{
			return vec2f(0,0);
		}

		virtual vec3f queryDiffuseImage(const vec2f &offset) const{
			return vec3f(0,0,0);
		}

		virtual vec3f queryNormalImage(const vec2f &offset, const vec3f &N) const{
			return vec3f(0,0,0);
		}

		virtual vec3f getDiffuse(const vec3f &worldPos) const{
			vec3f localPos = toLocal(worldPos);
			vec2f offset = getOffset(localPos);
			vec3f color = queryDiffuseImage(offset);
			return color;
		}

		virtual vec3f getNormal(const vec3f &worldPos, const vec3f &N) const{
			vec3f localPos = toLocal(worldPos);
			vec2f offset = getOffset(localPos);
			vec3f color = queryNormalImage(offset, N);
			return color;
		}

	};
}