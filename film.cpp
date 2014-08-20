#include "stdafx.h"
#include "film.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>



namespace LFJ{
	IplImage* convert_to_float32(IplImage* img)
	{
		IplImage* img32f = cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,img->nChannels);

		for(int i=0; i<img->height; i++)
		{
			for(int j=0; j<img->width; j++)
			{
				cvSet2D(img32f,i,j,cvGet2D(img,i,j));
			}
		}
		return img32f;
	}

	void Film::loadImage(const std::string &filename){
		IplImage *img_ori = cvLoadImage(filename.c_str(), 1);
		IplImage *img = convert_to_float32(img_ori);
		int height = img->height, width = img->width;
		const vec2f resolution = vec2f(width, height);
		this->setup(resolution);

		for(int x = 0; x < width; x++){
			for(int y = 0; y < height; y++){
				vec3f bgr = ((vec3f*)img->imageData)[y*width+x];
				vec3f rgb = vec3f(bgr.z, bgr.y, bgr.x);
				mImage[(mResY-y-1)*mResX + x] = rgb / 255.0;
			}
		}
		std::cout << "load image: " << filename << " done. " << std::endl;
		std::string name = filename + ".pfm";
		savePFM(name);
		cvReleaseImage(&img_ori);
		cvReleaseImage(&img);
	}

}

