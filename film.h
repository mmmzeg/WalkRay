#pragma once
#include <vector>
#include <string>
#include <fstream>
#include "macros.h"
////  o-----------------> x
////  |      width       |
////  |                  |
////  | height           |
////  |                  |
////  V------------------o
////  y
namespace LFJ{
	class Film{
		std::vector<vec3f> mImage;
		vec2f			   mResolution;
	public:
		int				   mResX;    // width
		int				   mResY;    // height
	public:
		Film()
		{}

		void setup(const vec2f &res){
			mResolution = res;
			mResX = int(mResolution.x);
			mResY = int(mResolution.y);
			mImage.resize(mResX * mResY);
			clear();
		}

		void clear(){
			memset(&mImage[0], 0, sizeof(vec3f) * mImage.size());
		}

		void setBuffer(const std::vector<vec3f> &pixelColors){
			for(int y = 0; y < mResY; y++){
				for(int x = 0; x < mResX; x++){
					mImage[y*mResX + x] = pixelColors[(mResY-y-1)*mResX + x];
				}
			}
		}
 
		void scale(float k){
			for(size_t i = 0; i < mImage.size(); i++){
				mImage[i] *= k;
			}
		}
		
		vec3f getColor(int x, int y) const{
			x = CLAMP(x, 0, mResX-1);
			y = CLAMP(y, 0, mResY-1);
			return mImage[y * mResX + x];
		}

		void setColor(int x, int y, const vec3f &c){
			x = CLAMP(x, 0, mResX-1);
			y = CLAMP(y, 0, mResY-1);
			mImage[y * mResX + x] = c;
		}

		void loadImage(const std::string &filename);
		/*************************************************/
		/*\brief: save image to disk		             */
		/*************************************************/

		/********************* ppm ***********************/
		void savePPM(
			const std::string &filename,
			float			   gamma = 1.f)
		{
			const float invGamma = 1.f / gamma;

			std::ofstream ppm(filename.c_str());
			ppm << "P3" << std::endl;
			ppm << mResX << " " << mResY << std::endl;
			ppm << "255" << std::endl;

			vec3f *ptr = &mImage[0];

			for(int y=0; y<mResY; y++)
			{
				for(int x=0; x<mResX; x++)
				{
					ptr = &mImage[x + y*mResX];
					int r = int(std::pow(ptr->x, invGamma) * 255.f);
					int g = int(std::pow(ptr->y, invGamma) * 255.f);
					int b = int(std::pow(ptr->z, invGamma) * 255.f);

					ppm << std::min(255, std::max(0, r)) << " "
						<< std::min(255, std::max(0, g)) << " "
						<< std::min(255, std::max(0, b)) << " ";
				}

				ppm << std::endl;
			}
		}

		/********************* pfm ***********************/
		void savePFM(
			const std::string &filename)
		{
			std::ofstream ppm(filename.c_str(), std::ios::binary);
			ppm << "PF" << std::endl;
			ppm << mResX << " " << mResY << std::endl;
			ppm << "-1" << std::endl;

			ppm.write(reinterpret_cast<const char*>(&mImage[0]),
				mImage.size() * sizeof(vec3f));
		}


		/********************* bmp ***********************/
		struct BmpHeader
		{
			uint   mFileSize;        // Size of file in bytes
			uint   mReserved01;      // 2x 2 reserved bytes
			uint   mDataOffset;      // Offset in bytes where data can be found (54)

			uint   mHeaderSize;      // 40B
			int    mWidth;           // Width in pixels
			int    mHeight;          // Height in pixels

			short  mColorPlates;     // Must be 1
			short  mBitsPerPixel;    // We use 24bpp
			uint   mCompression;     // We use BI_RGB ~ 0, uncompressed
			uint   mImageSize;       // mWidth x mHeight x 3B
			uint   mHorizRes;        // Pixels per meter (75dpi ~ 2953ppm)
			uint   mVertRes;         // Pixels per meter (75dpi ~ 2953ppm)
			uint   mPaletteColors;   // Not using palette - 0
			uint   mImportantColors; // 0 - all are important
		};
		void saveBMP(
			const std::string &filename,
			float			   gamma = 1.f)
		{
			std::ofstream bmp(filename.c_str(), std::ios::binary);
			BmpHeader header;
			bmp.write("BM", 2);
			header.mFileSize   = uint(sizeof(BmpHeader) + 2) + mResX * mResX * 3;
			header.mReserved01 = 0;
			header.mDataOffset = uint(sizeof(BmpHeader) + 2);
			header.mHeaderSize = 40;
			header.mWidth      = mResX;
			header.mHeight     = mResY;
			header.mColorPlates     = 1;
			header.mBitsPerPixel    = 24;
			header.mCompression     = 0;
			header.mImageSize       = mResX * mResY * 3;
			header.mHorizRes        = 2953;
			header.mVertRes         = 2953;
			header.mPaletteColors   = 0;
			header.mImportantColors = 0;

			bmp.write((char*)&header, sizeof(header));

			const float invGamma = 1.f / gamma;
			for(int y=mResY-1; y>=0; y--)
			{
				for(int x=0; x<mResX; x++)
				{
					// bmp is stored from bottom up
					const vec3f &rgbF = mImage[x + (mResY-y-1)*mResX];
					typedef unsigned char byte;
					float gammaBgr[3];
					gammaBgr[0] = std::pow(rgbF.z, invGamma) * 255.f;
					gammaBgr[1] = std::pow(rgbF.y, invGamma) * 255.f;
					gammaBgr[2] = std::pow(rgbF.x, invGamma) * 255.f;

					byte bgrB[3];
					bgrB[0] = byte(std::min(255.f, std::max(0.f, gammaBgr[0])));
					bgrB[1] = byte(std::min(255.f, std::max(0.f, gammaBgr[1])));
					bgrB[2] = byte(std::min(255.f, std::max(0.f, gammaBgr[2])));

					bmp.write((char*)&bgrB, sizeof(bgrB));
				}
			}
		}

		/********************* hdr ***********************/
		void saveHDR(
			const std::string &filename)
		{
			std::ofstream hdr(filename.c_str(), std::ios::binary);

			hdr << "#?RADIANCE" << '\n';
			hdr << "# SmallVCM" << '\n';
			hdr << "FORMAT=32-bit_rle_rgbe" << '\n' << '\n';
			hdr << "-Y " << mResY << " +X " << mResX << '\n';

			for(int y=0; y<mResY; y++)
			{
				for(int x=0; x<mResX; x++)
				{
					typedef unsigned char byte;
					byte rgbe[4] = {0,0,0,0};

					const vec3f &rgbF = mImage[x + y*mResX];
					float v = std::max(rgbF.x, std::max(rgbF.y, rgbF.z));

					if(v >= 1e-32f)
					{
						int e;
						v = float(frexp(v, &e) * 256.f / v);
						rgbe[0] = byte(rgbF.x * v);
						rgbe[1] = byte(rgbF.y * v);
						rgbe[2] = byte(rgbF.z * v);
						rgbe[3] = byte(e + 128);
					}

					hdr.write((char*)&rgbe[0], 4);
				}
			}
		}

	};
}