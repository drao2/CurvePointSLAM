
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
#define PATCH_SIZE      21
#define PATCH_HALF     ((PATCH_SIZE-1)/2)
#define EPI_THRESH      10
#define DISP_THRESH_HI      100
#define DISP_THRESH_LO      5
#define DISP_THRESH_CHANGE      25
#define SSD_THRESH      (0.5*WINDOW_SIZE*WINDOW_SIZE)//700.0
#define DA_SSD_THRESH   SSD_THRESH/2.0
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

#define BORDER_TOP     20
#define BORDER_LEFT     20
#define BORDER_RIGHT     300
#define BORDER_BOTTOM     220

#define HUGE_NUMBER     1000000.0
#define HUGER_NUMBER    10000000.0

#define LM_MOTION_THRESH   100.0     //The amount the landmark is allowed to drift in the image per frame
#define LM_PROXIMITY_THRESH     20.0    //For new landmarks, we reject them if they are this close to an existing one
#define NUM_CURRENT_LANDMARKS   10

#define MAX_PT_DIST     5.0     //The furthest distance away that the points can be

class PointFeaturesClass
{
public:
	PointFeaturesClass();
	~PointFeaturesClass();

	void getPointMeasurements(IplImage ** last_image, IplImage ** image, IplImage ** image_color, double * meas_new, int * num_new_meas, double * meas_existing, int * num_existing_meas, int * correspondence, double * planar_pose_meas);
        void getStereoPts(IplImage ** image, double * stereo_pts, int * num_features);
        void determineDataAssoc(IplImage ** image, double * measurements, int num_features, double * meas_new, int * num_new_meas, double * meas_existing, int * num_existing_meas, int * correspondence);
        //determinedataAssoc takes ALL measurements, finds associations and uses it to split into 'new' and 'existing'
        
        void trackExistingLandmarks(IplImage ** last_image, IplImage ** image, double * meas_existing, int *num_existing_meas, int * correspondence);
        void findNewLandmarks(IplImage ** image, double * meas_new, int *num_new_meas);
private:
        char optical_flow_found_feature_left[MAX_VODOM_FEATURES];
        float optical_flow_feature_error_left[MAX_VODOM_FEATURES];
        char optical_flow_found_feature_right[MAX_VODOM_FEATURES];
        float optical_flow_feature_error_right[MAX_VODOM_FEATURES];
        CvSize optical_flow_window;
        CvTermCriteria optical_flow_termination_criteria;
        

        IplImage *eig_image, *temp_image, *pyramid1, *pyramid2;
        
        struct landmarkStruct
        {
                IplImage * patches;
                std::vector<double> last_meas;
                int times_obs;
                int frames_since_obs;
        };
        std::vector<struct landmarkStruct> existing_landmarks;
        
        int stereo_disparity[NUM_CURRENT_LANDMARKS];
        
        CvPoint2D32f pts_curr[2][NUM_CURRENT_LANDMARKS];
        int matches_curr[NUM_CURRENT_LANDMARKS];
        int num_pts_curr;
        CvPoint2D32f pts_last[2][NUM_CURRENT_LANDMARKS];
        int matches_last[NUM_CURRENT_LANDMARKS];
        int num_pts_last;
        
        CvPoint2D32f temp_pts_curr[2][NUM_CURRENT_LANDMARKS];
        
        int num_landmarks_total;
        
         
    
        timeval t_start, t_stop;
        timeval start, stop;
        float elapsedTime;
};


#define CLEAR(x) memset (&(x), 0, sizeof (x))
	
