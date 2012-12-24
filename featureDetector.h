#ifndef FEATUREDETECTOR_H

#define FEATUREDETECTOR_H

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include "sys/time.h"
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>
#include "common.h"


//#define SHI_TOMASI
#define EDGE

#ifdef EDGE
    #define MAX_FEATURES        50000
#else
    #define MAX_FEATURES	50000
#endif
#define	NEIGHBOURHOOD_SIZE	31

#define MINZERO(X)		(((X) > 0.0) ? (X) : (0.0) )

#define EDGE_Y_CUTOFF           80

#define MEADOWBROOK
//#define CRYSTAL_LAKE
//#define QUAD

#define Y_CHOP ((3*PIC_HEIGHT)/5)
#define X_CHOP ((2*PIC_WIDTH)/5)

using namespace std;

struct Keypoint
{
	CvPoint image_pt;
	IplImage * neighbourhood;
	int stereo_match;
};

class FeatureDetector
{
public:
	FeatureDetector();
	~FeatureDetector();
	void find_features(IplImage* img, IplImage* seg_img, int cam, CvRect roi, CvPoint2D32f * map_endpts);
	void extract_neighbourhoods(CvPoint2D32f *points, int num_points, IplImage *img);
        void findFloodFillSeed(IplImage * edge_search_region, CvPoint * seed);
	//void edge_following(IplImage *img);
        void createEdgeSequences(IplImage *, IplImage *, CvRect roi);
        void createEdgeSequences(IplImage *, IplImage *, CvRect roi, CvPoint2D32f * map_endpts);
	void plot_features(IplImage* img);
	std::vector<CvPoint> ** return_features();
	float getTime();
	void resetTime();

private:
	CvSize size;
	IplImage* edge;
	IplImage* edge_clr;
	IplImage* eig_image;
        IplImage * edge_search_region;
        IplImage * last_edge_search_region;


        IplImage * img_hsv;
        IplImage * img_h;
        IplImage * img_s;
        IplImage * img_v;

        IplImage * img;


	CvPoint2D32f feature_points[MAX_FEATURES];
	CvMat * feature_neighbourhoods[MAX_FEATURES];
        
	int num_features;
	timeval start, stop;
	float elapsedTime;


        int prev_left_x, prev_left_y, prev_right_x, prev_right_y;
    int current_left_x, current_left_y, current_right_x, current_right_y;


        std::vector<CvPoint> featuresLeftCurve;
        std::vector<CvPoint> featuresRightCurve;
        std::vector<CvPoint> * features[2];
};

#endif

