#ifndef DISPLAYCLASS_H

#define DISPLAYCLASS_H

#include <stdlib.h>
#include <stdio.h>
//#include <cv.h>
//#include <cvaux.h>
//#include <highgui.h>
#include <string>
#include <iostream>
#include "sys/time.h"
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "common.h"
#include "featureDetector.h"

//#define OUTPUT_VIDEO

using namespace std;

class DisplayClass
{
public:
	DisplayClass();
	~DisplayClass();
        void convert3Dtogroundmap(CvPoint3D32f * world_point, CvPoint * map_point);
        void copy_images(IplImage ** image);
        void display_images(CvMat * featuresL, CvMat * featuresR);
        void display_images(IplImage* image_left, IplImage* image_right, vector<Keypoint> *features_left, vector<Keypoint> *features_right);
        void display_images(IplImage* image_left, IplImage* image_right, CvMat ** featuresLeftImage, CvMat ** featuresRightImage, double * p_left, double * p_right, IplImage * img_out_left, IplImage * img_out_right);
        void display_images(double * p_left, double * p_right);
        void generate_map(CvMat * state, std::vector<double> * state_limits, CvMat * z);
        
        float getTime();
	void resetTime();
	

private:
	timeval start, stop;
	float elapsedTime;
        IplImage * camera_images;
        IplImage * image_color_left;
        IplImage * image_color_right;
        IplImage * output_image;
        IplImage * cumulative_path;

        IplImage *landmark_map;

        CvVideoWriter* video_out;
};

#endif
