
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
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "common.h"

#include "levmar-2.5/levmar.h"
#include "levmar-2.5/misc.h"


#define MAX_VODOM_FEATURES    200
#define WINDOW_SIZE     21
#define WINDOW_HALF     ((WINDOW_SIZE-1)/2)
#define EPI_THRESH      5
#define DISP_THRESH      50
#define SSD_THRESH      (2.0*WINDOW_SIZE*WINDOW_SIZE)//700.0
#define LM_OPTS_SZ    	 5 /* max(4, 5) */
#define LM_INFO_SZ    	 10
#define LM_ERROR         -1
#define LM_INIT_MU    	 1E-03
#define LM_STOP_THRESH	 1E-17
#define LM_DIFF_DELTA    1E-06
#define LM_VERSION       "2.5 (December 2009)"

#define NUM_RANSAC_ITERATIONS   20
#define REPROJ_OUTLIER_THRESH   20.0
#define MIN_INLIER_PTS          10

#define HUGE_NUMBER     1000000.0
#define HUGER_NUMBER    10000000.0


void vodommodelfunc(double *params, double *, int m, int n, void *datax);
void stereo_image2body(double * stereo_pt, CvMat * body_pt);
void stereo_body2image(CvMat * body_pt, double * stereo_pt);


class VisualOdomClass
{
public:
	VisualOdomClass();
	~VisualOdomClass();

	void getPoseEstimate(IplImage ** last_image, IplImage ** current_image, double * params);
        void getStereoPts(IplImage ** last_image, IplImage ** current_image, double * last_stereo_pts, double * current_stereo_pts, int * num_features);
        void PoseFromRANSAC(double * last_stereo_pts, double * current_stereo_pts, int num_pts, double * params);
        double PoseFromNPts(double * last_stereo_pts, double * current_stereo_pts, double * params, int num_pts);
        void RandomSampleWithoutReplacement(int max_num, int num_to_choose, int * samples);
        double GetReprojectionError(double * last_stereo_pt, double * current_stereo_pt, double * params);
        void RemoveOutliers(double * last_stereo_pts, double * current_stereo_pts, int num_pts, double * params);
private:
        char optical_flow_found_feature[MAX_VODOM_FEATURES];
        float optical_flow_feature_error[MAX_VODOM_FEATURES];
        CvSize optical_flow_window;
        CvTermCriteria optical_flow_termination_criteria;
        

        IplImage *eig_image, *temp_image, *pyramid1, *pyramid2;
};


#define CLEAR(x) memset (&(x), 0, sizeof (x))
	
