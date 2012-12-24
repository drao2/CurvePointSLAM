#ifndef DATAASSOCCLASS_H

#define DATAASSOCCLASS_H

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
#include <sys/stat.h>
#include <sys/types.h>
#include "common.h"
#include "featureDetector.h"

#define NUM_PTS 1
#define NUM_VODOM_PTS 8

#define DATA_ASSOC_THRESHOLD    25.0

using namespace std;

class DataAssocClass
{
public:
	DataAssocClass();
	~DataAssocClass();

        double * getMatchT( CvMat * curve_features, IplImage ** last_image, IplImage ** last_image_track, IplImage ** image, IplImage ** last_image_color, IplImage ** image_color, double * last_params, double * params, int curve_num, char *data_assoc_flag, bool reset_data_assoc);
        void singleFrameTrack(IplImage ** last_image, IplImage ** image, CvRect roi);
        void updateMapCurves();

        float getTime();
	void resetTime();
	
        CvPoint2D32f map_endpt_tracked;

        CvPoint2D32f track_pts[NUM_TRACK_PTS];
        CvPoint2D32f track_pts_a[NUM_TRACK_PTS];
        
        
        CvPoint2D32f vodom_pts[NUM_VODOM_PTS];
        CvPoint2D32f vodom_pts_tracked[NUM_VODOM_PTS];
        
        double vodom_t_vals[NUM_VODOM_PTS][2];
private:
	timeval start, stop;
	float elapsedTime;

        bool first_time;

        CvPoint2D32f last_meas_endpt;
        CvPoint2D32f meas_endpt;
        CvPoint2D32f last_meas_startpt;
        CvPoint2D32f meas_startpt;


        CvPoint2D32f last_meas_endpt_match;
        CvPoint2D32f meas_endpt_match;
        CvPoint2D32f last_meas_startpt_match;
        CvPoint2D32f meas_startpt_match;

        CvPoint2D32f map_endpt;
        CvPoint2D32f map_endpt_match;

                double t_split[3];
                double last_t_split[3];
                double last_m;
                double last_b;
                double current_m;
                double current_b;

        int curve_left_end_num_pt;
        int curve_right_end_num_pt;
        int curve_left_start_num_pt;
        int curve_right_start_num_pt;
        bool tracking_started;


        CvPoint2D32f temp_pt;
        CvPoint2D32f temp_pt2;
        CvPoint2D32f temp_pt3;
        CvPoint2D32f temp_pt4;
        CvPoint2D32f temp_pt5;
        CvPoint2D32f temp_pt6;
        CvPoint2D32f temp_pt2a;
        CvPoint2D32f temp_pt3a;
        CvPoint2D32f temp_pt4a;
        CvPoint2D32f temp_pt5a;
        CvPoint2D32f temp_pt6a;
        CvPoint2D32f pt_diff;
        
        
    CvMat * A;
    CvMat * At;
    CvMat * AtA;
    CvMat * AtAinv;
    CvMat * b;
    CvMat * xparams;

        char optical_flow_found_feature[1];
    float optical_flow_feature_error[1];
    CvSize optical_flow_window;
    CvTermCriteria optical_flow_termination_criteria;

    CvSize frame_size;

    IplImage *eig_image, *temp_image, *pyramid1, *pyramid2;


};


#endif
