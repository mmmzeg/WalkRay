#pragma once
#include "textureMap.h"
#include "frame.h"

namespace LFJ{
	class TexturePlaneMap : public TextureMap{
	public:
		bool use_normal_map;
	public:
		TexturePlaneMap(const mat4f t, vec3f p_min, vec3f p_max):
			TextureMap(t, p_min, p_max) 
		{}
		virtual vec2f getOffset(const vec3f &localPos) const{
			vec2f offset(0,0);

			vec3f mP0ToP1 = p1 - p0;
			vec3f mP0ToLocalPos = localPos - p0;
			float boundX = mP0ToP1.x, mX = mP0ToLocalPos.x,
				  boundY = mP0ToP1.y, mY = mP0ToLocalPos.y,
				  boundZ = mP0ToP1.z, mZ = mP0ToLocalPos.z;

			if(boundX == 0){
				int w = mZ / boundZ * normalMap.mResX;
				int h = mY / boundY * normalMap.mResY;
				if(w < 0 || w >= normalMap.mResX || h < 0 || h >= normalMap.mResY){
					return vec2f(0,0);
				}
				return vec2f(w, h);
			}
			if(boundY == 0){
				int w = mX / boundX * normalMap.mResX;
				int h = mZ / boundZ * normalMap.mResY;
				if(w < 0 || w >= normalMap.mResX || h < 0 || h >= normalMap.mResY){
					return vec2f(0,0);
				}
				return vec2f(w, h);
			}
			if(boundZ == 0){
				int w = mX / boundX * normalMap.mResX;
				int h = mY / boundY * normalMap.mResY;
				if(w < 0 || w >= normalMap.mResX || h < 0 || h >= normalMap.mResY){
					return vec2f(0,0);
				}
				return vec2f(w, h);
			}
			std::cerr << "Only orthogonal plane supported currently. " << std::endl;

			return vec2f(0,0);
		}

		virtual vec3f queryDiffuseImage(const vec2f &offset) const{
			vec3f diffuseColor = diffuseMap.getColor(offset.x, offset.y);
			return diffuseColor;
		}

		virtual vec3f queryNormalImage(const vec2f &offset, const vec3f &N) const{
		if(!use_normal_map){
			
			vec3f color_O = normalMap.getColor(offset.x, offset.y), 
				  color_O_dX = normalMap.getColor(offset.x + 1, offset.y), 
				  color_O_dY = normalMap.getColor(offset.x, offset.y + 1);
			
			Frame frame;
			frame.setFromN(N);

			double dx = (color_O_dX - color_O).length(), dy = (color_O_dY - color_O).length();
			
			vec3f norm = vec3f(0,1,0) + vec3f(1,0,0) * dx + vec3f(0,0,1) * dy;
			norm.normalize();
			norm = frame.toWorldFrame(norm);

			return norm;
		}
		else{
			vec3f gradientO = normalMap.getColor(offset.x, offset.y); 
			
			int nIndex = 0;
			for(int i = 1; i < 3; i++){
				if(gradientO[i] > gradientO[0]){
					nIndex = i;
				}
			}
			std::swap(gradientO[1], gradientO[nIndex]);
			
			Frame frame;
			frame.setFromN(N);


			vec3f norm = gradientO;

			norm.normalize();
			norm = frame.toWorldFrame(norm);

			return norm;
		}
		}


		virtual vec3f getDiffuse(const vec3f &worldPos) const{
			vec3f localPos = toLocal(worldPos);
			vec2f offset = getOffset(localPos);
			vec3f color = queryDiffuseImage(offset);
			return color;
		}

		virtual vec3f getNormal(const vec3f &worldPos, const vec3f &N) const{
		//!!!!!
		//	return N;
			
			
			
			vec3f localPos = toLocal(worldPos);
			vec2f offset = getOffset(localPos);
			vec3f norm = queryNormalImage(offset, N);
			return norm;


			norm.normalize();

			vec3f up = vec3f(0,1,0);
			vec3f axis = up.cross(N);
			axis.normalize();
			float angle = acos(up.dot(N));
			norm = vec3f(RotateMatrix(axis, angle) * vec4f(norm, 0));
			
			return norm;
		}

	};
}
