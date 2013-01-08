#include <stdlib.h>
#include <stdio.h>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <sys/time.h>
#include "common.h"
#include "kalmanClass.h"
#include "capture.h"
#include "featureDetector.h"
#include "displayClass.h"
#include "dataAssocClass.h"
#include "curveFittingClass.h"
#include "pointFeaturesClass.h"

#define ALPHA   1.0

#ifdef QUAD
        #define LEFT_IMAGE_LAG -2
#else
    #ifdef MEADOWBROOK
        #define LEFT_IMAGE_LAG 0
    #else
        #ifdef CRYSTAL_LAKE
            #define LEFT_IMAGE_LAG 1
        #else
                #define LEFT_IMAGE_LAG  2
        #endif
    #endif
#endif
#define LEFT_RIGHT_OFFSET   0
//#define LEFT_RIGHT_OFFSET   2
//#define LEFT_RIGHT_OFFSET   -2

#define ERROR_THRESHOLD 50000.0
//#define ERROR_THRESHOLD 23000.0

#define FRAME_INTERVAL  1
#define TOP_Y_CUTOFF    50
//#define FRAMES_TO_SKIP  3101
#define FRAMES_TO_SKIP  150
#define LENGTH_EACH_CURVE  75
#define NUM_FRAMES_DA_RESET     10
using namespace std;

void mouse_left(int event, int x, int y, int flags, void * param);
void mouse_right(int event, int x, int y, int flags, void * param);
bool left_window_click, right_window_click, allow_callback;
void mouse_left(int event, int x, int y, int flags, void * param)
{
    if (allow_callback)
    {
        switch (event)
        {
            case CV_EVENT_LBUTTONDOWN:
            {
                cout << "Left: " << x << "\t" << y << endl;
                break;
            }
            case CV_EVENT_LBUTTONUP:
            {
                break;
            }
            case CV_EVENT_MOUSEMOVE:
            {
                break;
            }
            default:
            {
                break;
            }
        }
    }
}
void mouse_right(int event, int x, int y, int flags, void * param)
{
    if (allow_callback)
    {
        switch (event)
        {
            case CV_EVENT_LBUTTONDOWN:
            {
                cout << "Right: " << x << "\t" << y << endl;
                break;
            }
            case CV_EVENT_LBUTTONUP:
            {
                break;
            }
            case CV_EVENT_MOUSEMOVE:
            {
                break;
            }
            default:
            {
                break;
            }
        }
    }
}
void plotGraph(double * y, int num_pts, int width, int height)
{
    cvNamedWindow("Graph1");
    cvMoveWindow("Graph1",0,0);

    IplImage * graph_image = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);

    double y_max = 0.0;
    for (int i = 0; i < num_pts; i++)
    {
        if (y[i] > y_max)            y_max = 45.0;//y[i];
    }

        cvLine(graph_image, cvPoint(0,height/2),cvPoint(width,height/2),CV_RGB(255,255,255),1,8,0);
    for (int i = 0; i < num_pts; i++)
    {
        CvPoint plot_point = cvPoint(i*width/num_pts, height/2-0.75*y[i]*height/y_max);
        cvCircle(graph_image, plot_point,1,CV_RGB(255,0,0));
    }

    cvShowImage("Graph1",graph_image);
    cvWaitKey(10);

}


#include <iostream>
#include <fstream>

timeval t_start, t_stop;
timeval start, stop;
float elapsedTime;
unsigned int binom[50][50];

CvPoint last_pts[2];
CvPoint current_pts[2];

int main(void)
{
        int cam_index[2];
        cam_index[LEFT] = 1;
        cam_index[RIGHT] = 2;

        CaptureDevice *camera = new CaptureDevice[NUM_CAMERAS];
	DisplayClass *display_GUI = new DisplayClass;
        KalmanFilter *EKF = new KalmanFilter;
        CurveFittingClass * curveFitter = new CurveFittingClass;
        DataAssocClass * curveMatcher = new DataAssocClass[2];
	FeatureDetector *features = new FeatureDetector[NUM_CAMERAS];
        PointFeaturesClass *pointFeatures = new PointFeaturesClass;

	IplImage * image[NUM_CAMERAS];
	IplImage * image_raw[NUM_CAMERAS];
	IplImage * last_image_raw[NUM_CAMERAS];
	IplImage * seg[NUM_CAMERAS];
	IplImage * image_color[NUM_CAMERAS];
	IplImage * last_image_color[NUM_CAMERAS];
	IplImage * last_image[NUM_CAMERAS];
	IplImage * last_image_track[NUM_CAMERAS];
        
        //Stereo rectification
        CvSize imageSize = cvSize(PIC_WIDTH,PIC_HEIGHT);
        CvMat * mx[2], * my[2];
        mx[0] = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        my[0] = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        mx[1] = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        my[1] = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* disp = cvCreateMat( imageSize.height,
            imageSize.width, CV_16S );
        CvRect roi[2];
        
        CvMat *M1 = cvCreateMat(3, 3, CV_64F);
        CvMat *M2 = cvCreateMat(3, 3, CV_64F);
        CvMat *D1 = cvCreateMat(1, 5, CV_64F);
        CvMat *D2 = cvCreateMat(1, 5, CV_64F);
        CvMat *R = cvCreateMat(3, 3, CV_64F);
        CvMat *T = cvCreateMat(3, 1, CV_64F);
        CvMat *R1 = cvCreateMat(3, 3, CV_64F);
        CvMat *R2 = cvCreateMat(3, 3, CV_64F);
        CvMat *P1 = cvCreateMat(3, 4, CV_64F);
        CvMat *P2 = cvCreateMat(3, 4, CV_64F);
        CvMat *Q = cvCreateMat(4, 4, CV_64F);
        cvSetZero(M1);
        cvSetZero(M2);
        cvSetZero(D1);
        cvSetZero(D2);
        cvSetZero(R1);
        cvSetZero(R2);
        cvSetZero(P1);
        cvSetZero(P2);
        cvSetZero(R);
        cvSetZero(T);
        cvSetZero(Q);
        
        M1->data.db[0] = FX0;
        M1->data.db[2] = CX0;
        M1->data.db[4] = FY0;
        M1->data.db[5] = CY0;
        M2->data.db[0] = FX1;
        M2->data.db[2] = CX1;
        M2->data.db[4] = FY1;
        M2->data.db[5] = CY1;
        R->data.db[0] = R11;
        R->data.db[1] = R12;
        R->data.db[2] = R13;
        R->data.db[3] = R21;
        R->data.db[4] = R22;
        R->data.db[5] = R23;
        R->data.db[6] = R31;
        R->data.db[7] = R32;
        R->data.db[8] = R33;
        T->data.db[0] = -T1;
        T->data.db[1] = -T2;
        T->data.db[2] = -T3;
        D1->data.db[0] = D01;
        D1->data.db[1] = D02;
        D1->data.db[2] = D03;
        D1->data.db[3] = D04;
        D1->data.db[4] = D05;
        D2->data.db[0] = D11;
        D2->data.db[1] = D12;
        D2->data.db[2] = D13;
        D2->data.db[3] = D14;
        D2->data.db[4] = D15;

            cvStereoRectify( M1, M2, D1, D2, imageSize,
                R, T,
                R1, R2, P1, P2, Q,
                CV_CALIB_ZERO_DISPARITY,
                1, imageSize, &(roi[0]), &(roi[1]));
                        
                //roi[1].x += imageSize.width;
    //Precompute maps for cvRemap()
            cvInitUndistortRectifyMap(M1,D1,R1,P1,mx[0],my[0]);
            cvInitUndistortRectifyMap(M2,D2,R2,P2,mx[1],my[1]);

	for (int i = 0; i < NUM_CAMERAS; ++i)
	{
		camera[i].init_capture(cam_index[i]);	//+1 is so we dont use the laptop webcam
	}


//OPTICAL FLOW INIT
        CvSize frame_size = cvSize(PIC_WIDTH,PIC_HEIGHT);
        last_image[0] = cvCreateImage( frame_size, IPL_DEPTH_8U, 1 );
        last_image[1] = cvCreateImage( frame_size, IPL_DEPTH_8U, 1 );
        last_image_raw[0] = cvCreateImage( frame_size, IPL_DEPTH_8U, 3 );
        last_image_raw[1] = cvCreateImage( frame_size, IPL_DEPTH_8U, 3 );
        last_image_track[0] = cvCreateImage( frame_size, IPL_DEPTH_8U, 1 );
        last_image_track[1] = cvCreateImage( frame_size, IPL_DEPTH_8U, 1 );
        image_color[0] = cvCreateImage( frame_size, IPL_DEPTH_8U, 3 );
        image_color[1] = cvCreateImage( frame_size, IPL_DEPTH_8U, 3 );
        image[0] = cvCreateImage( frame_size, IPL_DEPTH_8U, 1 );
        image[1] = cvCreateImage( frame_size, IPL_DEPTH_8U, 1 );
        last_image_color[0] = cvCreateImage( frame_size, IPL_DEPTH_8U, 3 );
        last_image_color[1] = cvCreateImage( frame_size, IPL_DEPTH_8U, 3 );
        
        cvNamedWindow("Left");
        cvNamedWindow("Right");
        cvMoveWindow("Left",0,0);
        cvMoveWindow("Right",0,0);
        cvSetMouseCallback( "Left", mouse_left, NULL );
        cvSetMouseCallback( "Right", mouse_right, NULL );

        left_window_click = false;
        right_window_click = false;
        allow_callback = false;

//Get frame from camera
        for (int n = 0; n < FRAMES_TO_SKIP; n++)
        {
            for (int i = 0; i < NUM_CAMERAS; ++i)
            {
                    image_raw[i] = camera[i].get_frame();
            }
        }
        for (int i = 0; i < LEFT_IMAGE_LAG; ++i)
        {
                image_raw[0] = camera[0].get_frame();
        }
        if (LEFT_IMAGE_LAG < 0)
        {
            for (int i = 0; i < -LEFT_IMAGE_LAG; ++i)
            {
                    image_raw[1] = camera[1].get_frame();
            }
        }

            cvRemap(image_raw[0], image_color[0], mx[0], my[0]);
            cvRemap(image_raw[1], image_color[1], mx[1], my[1]);
            //cvCopy(image_color[0],image_raw[0],NULL);
            //cvCopy(image_color[1],image_raw[1],NULL);
            cvCopy(image_raw[0],image_color[0],NULL);
            cvCopy(image_raw[1],image_color[1],NULL);

        //Initialise binomial lookuptable
        for (int n = 0; n < 50; n++)
        {
            for (int k = 0; k < 50; k++)
            {
                if (k == 0)
                    binom[n][k] = 1;
                else if (n == 0)
                    binom[n][k] = 0;
                else
                    binom[n][k] = binom[n-1][k]+binom[n-1][k-1];
            }
        }

        //Init RNG
        CvRandState randstate;
        cvRandInit(&randstate,0.1,0.0,0xffffffff,CV_RAND_NORMAL);
        
        CvMat *z8 = cvCreateMat(8,1,CV_64FC1);
        CvMat *z16 = cvCreateMat(19,1,CV_64FC1);
        CvMat *z = cvCreateMat(35,1,CV_64FC1);
        CvMat *z1 = cvCreateMat(4,1,CV_64FC1);
        CvMat *z2 = cvCreateMat(4,1,CV_64FC1);
        CvMat *z3 = cvCreateMat(4,1,CV_64FC1);
        CvMat *z4 = cvCreateMat(4,1,CV_64FC1);
        CvMat *z_left = cvCreateMat(16,1,CV_64FC1);
        CvMat *z_right = cvCreateMat(16,1,CV_64FC1);
        
        std::vector<double> state_limits;


        CvMat * A1 = cvCreateMat(4,4,CV_64FC1);
        CvMat * A2 = cvCreateMat(4,4,CV_64FC1);
	CvMat * A1l = cvCreateMat(4,4,CV_64FC1);
	CvMat * A2l = cvCreateMat(4,4,CV_64FC1);
	CvMat * B1l = cvCreateMat(4,4,CV_64FC1);
	CvMat * B2l = cvCreateMat(4,4,CV_64FC1);
	CvMat * A1r = cvCreateMat(4,4,CV_64FC1);
	CvMat * A2r = cvCreateMat(4,4,CV_64FC1);
	CvMat * B1r = cvCreateMat(4,4,CV_64FC1);
	CvMat * B2r = cvCreateMat(4,4,CV_64FC1);
        CvMat * I4 = cvCreateMat(4,4,CV_64FC1);
        cvSetZero(I4);
        I4->data.db[0] = 1.0;
        I4->data.db[5] = 1.0;
        I4->data.db[10] = 1.0;
        I4->data.db[15] = 1.0;

        int counter = 0;
        
        double params[19];
        for (int i = 0; i < 16; i++)
            params[i] = 0.0;
        params[16] = 0.0;
        params[17] = -0.2;
        params[18] = -2.0;
        double last_params[19];
        for (int i = 0; i < 16; i++)
            params[i] = 0.0;
        params[16] = 0.0;
        params[17] = -0.2;
        params[18] = -2.0;

                
        char left_assoc_flag = ADD_FIRST_STATES;
        char right_assoc_flag = ADD_FIRST_STATES;
        bool partial_curve_observed[2] = {false, false};
        bool first_time = true;

        std::vector<int> left_curve_nums;
        std::vector<int> right_curve_nums;
        std::vector<int> curves_updated_last;
        curves_updated_last.push_back(0);
        curves_updated_last.push_back(1);
        std::vector<int> curves_to_update;
        int num_map_curves = 0;
        
        double t_split[6];
        double last_t_split[6];

        bool edges_detected = true;
        bool reset_data_assoc = true;
        bool valid_measurement = true;
        bool valid_last_measurement = true;
        int frames_since_good_measurement = 0;

        //Vodom Pose variables
        double vodom_params[6] = {0.0};
        CvMat * R_12 = cvCreateMat(3,3,CV_64FC1);
        cvSetZero(R_12);
        cvSetZero(R_12);
        R_12->data.db[0] = 1.0;
        R_12->data.db[4] = 1.0;
        R_12->data.db[8] = 1.0;
        CvMat * t_12 = cvCreateMat(3,1,CV_64FC1);
        cvSetZero(t_12);
       

        while(1)
	{
            if(valid_measurement)
                frames_since_good_measurement = 1;
            else
                frames_since_good_measurement++;
            if (edges_detected = false || first_time || frames_since_good_measurement >= NUM_FRAMES_DA_RESET)
                reset_data_assoc = true;
            else
                reset_data_assoc = false;
            
            valid_last_measurement = valid_measurement;

            cvCopy(image_raw[0], last_image_raw[0], NULL);
            cvCopy(image_raw[1], last_image_raw[1], NULL);
            cvCvtColor(last_image_raw[0],last_image[0],CV_RGB2GRAY);
            cvCvtColor(last_image_raw[1],last_image[1],CV_RGB2GRAY);
            //cvCopy(last_image_raw[0], last_image_color[0], NULL);
            //cvCopy(last_image_raw[1], last_image_color[1], NULL);
            cvCopy(image_color[0], last_image_color[0], NULL);
            cvCopy(image_color[1], last_image_color[1], NULL);

            //Do the REAL VISION stuff first
            for (int n = 0; n < FRAME_INTERVAL; n++)
            {
		for (int i = 0; i < NUM_CAMERAS; ++i)
		{
                        cvCopy(image[i], last_image_track[i],NULL);
			image_raw[i] = camera[i].get_frame();
                        cvRemap(image_raw[i], image_color[i], mx[i], my[i]);
                        cvCopy(image_color[i],image_raw[i],NULL);
                        //cvCopy(image_raw[i],image_color[i],NULL);
                        cvCvtColor(image_raw[i],image[i],CV_RGB2GRAY);
                }
                
                //Track each curve too (if last measurement was valid)
                if (!reset_data_assoc)
                {
                        curveMatcher[0].singleFrameTrack(last_image_track,image, roi[0]);
                        curveMatcher[1].singleFrameTrack(last_image_track,image, roi[0]);
                }
                counter++;
                //if (!first_time)
                    //EKF->PredictKF();
            }
            int n_pts_existing = 0;
            int n_pts_new = 0;
            double point_meas_existing[200];
            double point_meas_new[200];
            int correspondences[50];
            
            cvCopy(image_raw[0], image_color[0], NULL);
            cvCopy(image_raw[1], image_color[1], NULL);
            if(!first_time)
            {
                pointFeatures->getPointMeasurements(&last_image[0],&image[0], &image_color[0], &point_meas_new[0],&n_pts_new,&point_meas_existing[0],&n_pts_existing, &correspondences[0]);
                //cout << n_pts_existing << " " << n_pts_new << endl;
                double phi = vodom_params[3];
                double theta = vodom_params[4];
                double psi = vodom_params[5];
                t_12->data.db[0] = vodom_params[0];
                t_12->data.db[1] = vodom_params[1];
                t_12->data.db[2] = vodom_params[2];
                generate_Reb(phi, theta, psi, R_12);
                //EKF->PredictKF(R_12,t_12);
                EKF->PredictKF();
            }
            
            //Add any new point measurements we've made
            EKF->AddNewPoints(&point_meas_new[0], n_pts_new);
            
            //cvRemap(image_raw[0], image_color[0], mx[0], my[0]);
            //cvRemap(image_raw[1], image_color[1], mx[1], my[1]);




//Find features in camera images
            CvPoint2D32f map_endpts[] = {curveMatcher[0].map_endpt_tracked,curveMatcher[1].map_endpt_tracked};
            for (int i = 0; i < NUM_CAMERAS; ++i)
            {
                    features[i].find_features(image_raw[i],seg[i],i,roi[i],&map_endpts[0]);
                    curveMatcher[i].map_endpt_tracked = map_endpts[i];
            }

            //cvResetImageROI(image_color[0]);
            //cvResetImageROI(image_color[1]);



#ifndef NO_SLAM
            
            display_GUI->copy_images(&(image[0]));

            for (int i = 0; i < 19; i++)
                last_params[i] = params[i];



//PROCESS POINTS AND FIT CURVE
            std::vector<CvPoint> ** featuresL =features[LEFT].return_features();
            std::vector<CvPoint> ** featuresR =features[RIGHT].return_features();


                CvMat * featuresLeftImage[2];
                CvMat * featuresRightImage[2];
                int left_y_cutoff[] = {MIN(TOP_Y_CUTOFF,curveMatcher[0].map_endpt_tracked.y),MIN(TOP_Y_CUTOFF,curveMatcher[1].map_endpt_tracked.y)};
                if(first_time)
                {
                    left_y_cutoff[0] = TOP_Y_CUTOFF;
                    left_y_cutoff[1] = TOP_Y_CUTOFF;                    
                }
                int RangeLeftImage[2][2] = { {0, featuresL[0]->size()-1}, {0, featuresL[1]->size()-1} };
                int RangeRightImage[2][2] = { {0, featuresR[0]->size()-1}, {0, featuresR[1]->size()-1} };


            if (featuresL[0]->size() < 10 || featuresL[1]->size() < 10 || featuresR[0]->size() < 10 || featuresR[1]->size() < 10)
                edges_detected = false;
            else
            {
                edges_detected = true;
                int ystartLeftImage[] = {featuresL[0]->at(0).y,featuresL[1]->at(0).y};
                int yendLeftImage[] = {featuresL[0]->at(featuresL[0]->size()-1).y,featuresL[1]->at(featuresL[1]->size()-1).y};
                int ystartRightImage[] = {featuresR[0]->at(0).y,featuresR[1]->at(0).y};
                int yendRightImage[] = {featuresR[0]->at(featuresR[0]->size()-1).y,featuresR[1]->at(featuresR[1]->size()-1).y};

                //Trim edge sequences so they match up and aren't too long!!!!
                for (int i = 0; i < 2; i++)
                {
                    //If left curve in left image starts lower, then remove edge points from start from left image
                    if (ystartLeftImage[i] > ystartRightImage[i]-LEFT_RIGHT_OFFSET && edges_detected)
                    {
                        while(ystartLeftImage[i] > ystartRightImage[i]-LEFT_RIGHT_OFFSET)
                        {
                            RangeLeftImage[i][0]++;

                            if (RangeLeftImage[i][0] >= featuresL[i]->size())
                            {
                                edges_detected = false;
                                break;
                            }
                            ystartLeftImage[i] = featuresL[i]->at(RangeLeftImage[i][0]).y;
                        }

                    }
                    //If right curve in left image starts lower, then remove edge points from start from right image
                    else if (ystartLeftImage[i] < ystartRightImage[i]-LEFT_RIGHT_OFFSET && edges_detected)
                    {
                        while(ystartLeftImage[i] < ystartRightImage[i]-LEFT_RIGHT_OFFSET)
                        {
                            RangeRightImage[i][0]++;

                            if (RangeRightImage[i][0] >= featuresR[i]->size())
                            {
                                edges_detected = false;
                                break;
                            }
                            ystartRightImage[i] = featuresR[i]->at(RangeRightImage[i][0]).y;
                        }

                    }
                }

                for (int i = 0; i < 2; i++)
                {
                    //If left curve in left image starts lower, then remove edge points from start from left image
                    if (yendLeftImage[i] < yendRightImage[i]-LEFT_RIGHT_OFFSET && edges_detected)
                    {
                        while(yendLeftImage[i] < yendRightImage[i]-LEFT_RIGHT_OFFSET)
                        {
                            RangeLeftImage[i][1]--;

                            if (RangeLeftImage[i][1] < 0)
                            {
                                edges_detected = false;
                                break;
                            }
                            yendLeftImage[i] = featuresL[i]->at(RangeLeftImage[i][1]).y;
                        }

                    }
                    //If left curve in right image starts lower, then remove edge points from start from right image
                    else if (yendLeftImage[i] > yendRightImage[i]-LEFT_RIGHT_OFFSET && edges_detected)
                    {
                        while(yendLeftImage[i] > yendRightImage[i]-LEFT_RIGHT_OFFSET)
                        {
                            RangeRightImage[i][1]--;

                            if (RangeRightImage[i][1] < 0)
                            {
                                edges_detected = false;
                                break;
                            }
                            yendRightImage[i] = featuresR[i]->at(RangeRightImage[i][1]).y;
                        }
                    }

                    while(yendLeftImage[i] < ystartLeftImage[i]-LENGTH_EACH_CURVE && edges_detected)
                    {
                        RangeLeftImage[i][1]--;

                        if (RangeLeftImage[i][1] < 0 )
                        {
                            edges_detected = false;
                            break;
                        }
                        yendLeftImage[i] = featuresL[i]->at(RangeLeftImage[i][1]).y;
                    }
                    while(yendRightImage[i] < ystartRightImage[i]-LENGTH_EACH_CURVE && edges_detected)
                    {
                        RangeRightImage[i][1]--;

                        if (RangeRightImage[i][1] < 0 )
                        {
                            edges_detected = false;
                            break;
                        }
                        yendRightImage[i] = featuresR[i]->at(RangeRightImage[i][1]).y;
                    }


                    while(yendLeftImage[i] > curveMatcher[i].map_endpt_tracked.y && !reset_data_assoc)
                    {
                        RangeLeftImage[i][0]++;
                        RangeLeftImage[i][1]++;

                        if (RangeLeftImage[i][0] >= featuresL[i]->size() || RangeLeftImage[i][1] >= featuresL[i]->size() )
                        {
                            edges_detected = false;
                            break;
                        }
                        yendLeftImage[i] = featuresL[i]->at(RangeLeftImage[i][1]).y;
                    }
                    while(yendRightImage[i] > curveMatcher[i].map_endpt_tracked.y + LEFT_RIGHT_OFFSET && !reset_data_assoc)
                    {
                        RangeRightImage[i][1]++;
                        RangeRightImage[i][0]++;

                        if (RangeRightImage[i][0] >= featuresR[i]->size() || RangeRightImage[i][1] >= featuresR[i]->size() )
                        {
                            edges_detected = false;
                            break;
                        }
                        yendRightImage[i] = featuresR[i]->at(RangeRightImage[i][1]).y;
                    }

                }
                if(yendLeftImage[0] > yendLeftImage[1])
                {
                    while(yendLeftImage[0] > yendLeftImage[1])
                    {
                        RangeLeftImage[0][0]++;
                        RangeLeftImage[0][1]++;

                            if (RangeLeftImage[0][0] >= featuresL[0]->size() || RangeLeftImage[0][1] >= featuresL[0]->size() )
                            {
                                edges_detected = false;
                                break;
                            }
                        yendLeftImage[0] = featuresL[0]->at(RangeLeftImage[0][1]).y;
                    }
                }
                else
                {
                    while(yendLeftImage[0] < yendLeftImage[1])
                    {
                        RangeLeftImage[1][0]++;
                        RangeLeftImage[1][1]++;

                            if (RangeLeftImage[1][0] >= featuresL[1]->size() || RangeLeftImage[1][1] >= featuresL[1]->size() )
                            {
                                edges_detected = false;
                                break;
                            }
                        yendLeftImage[1] = featuresL[1]->at(RangeLeftImage[1][1]).y;
                    }
                }

                if (yendRightImage[0] > yendRightImage[1])
                {
                    while(yendRightImage[0] > yendRightImage[1])
                    {
                        RangeRightImage[0][0]++;
                        RangeRightImage[0][1]++;

                        if (RangeRightImage[0][0] >= featuresR[0]->size() || RangeRightImage[0][1] >= featuresR[0]->size() )
                        {
                            edges_detected = false;
                            break;
                        }
                        yendRightImage[0] = featuresR[0]->at(RangeRightImage[0][1]).y;
                    }
                }
                else
                {
                    while(yendRightImage[0] < yendRightImage[1])
                    {
                        RangeRightImage[1][0]++;
                        RangeRightImage[1][1]++;

                        if (RangeRightImage[1][0] >= featuresR[1]->size() || RangeRightImage[1][1] >= featuresR[1]->size() )
                        {
                            edges_detected = false;
                            break;
                        }
                        yendRightImage[1] = featuresR[1]->at(RangeRightImage[1][1]).y;
                    }
                }
            }

            CvMat * state_current = EKF->getState();
            if(edges_detected)
            {
                //Copy features into CvMat form
                for (int n = 0; n < 2; n++)
                {
                    featuresLeftImage[n] = cvCreateMat(2*MIN(RangeLeftImage[n][1]-RangeLeftImage[n][0]+1,10000),1,CV_64FC1);
                    featuresRightImage[n] = cvCreateMat(2*MIN(RangeRightImage[n][1]-RangeRightImage[n][0]+1,10000),1,CV_64FC1);
                }


                for (int n = 0; n < 2; n++)
                {
                    for(int i = 0; i < featuresLeftImage[n]->rows/2; i++)
                    {
                        featuresLeftImage[n]->data.db[2*i] = featuresL[n]->at(RangeLeftImage[n][0]+i).x;
                        featuresLeftImage[n]->data.db[2*i+1] = featuresL[n]->at(RangeLeftImage[n][0]+i).y;
                    }
                    for(int i = 0; i < featuresRightImage[n]->rows/2; i++)
                    {
                        featuresRightImage[n]->data.db[2*i] = featuresR[n]->at(RangeRightImage[n][0]+i).x;
                        featuresRightImage[n]->data.db[2*i+1] = featuresR[n]->at(RangeRightImage[n][0]+i).y;
                    }
                }

                double euler[3], euler_last[3];
                double translation[3], translation_last[3];
                double p[16], p_left[16], p_right[16];
                double p_last[16], p_left_last[16], p_right_last[16];

                curveFitter->fit_curve(&(params[0]), featuresLeftImage, featuresRightImage, display_GUI);
                //valid_measurement = EKF->CheckValidMeasurement(params[16],params[17],params[18],frames_since_good_measurement);
                if (curveFitter->fitting_error > ERROR_THRESHOLD)
                    valid_measurement = false;

                for (int i = 0; i < 16; i++)
                {
                    if (fabs(params[i]) > 50.0)
                    {
                        valid_measurement = false;
                        break;
                    }
                }
                if (params[18] > 0.0)
                    valid_measurement = false;

                if(first_time)
                    valid_measurement = true;

                        double theta = params[16];
                        double phi = params[17];
                        double height = params[18];


                euler[0] = 0.0;
                euler[1] = params[16];
                euler[2] = params[17];
                translation[0] = 0.0;
                translation[1] = 0.0;
                translation[2] = params[18];

                control2coeffs(params,p);
                control2coeffs(&(params[8]),&(p[8]));
                poly_earth2image(&(p[0]), p_left, p_right, euler, translation);
                poly_earth2image(&(p[8]), &(p_left[8]), &(p_right[8]), euler, translation);

                euler_last[0] = 0.0;
                euler_last[1] = last_params[16];
                euler_last[2] = last_params[17];
                translation_last[0] = 0.0;
                translation_last[1] = 0.0;
                translation_last[2] = last_params[18];

                control2coeffs(last_params,p_last);
                control2coeffs(&(last_params[8]),&(p_last[8]));
                poly_earth2image(&(p_last[0]), p_left_last, p_right_last, euler_last, translation_last);
                poly_earth2image(&(p_last[8]), &(p_left_last[8]), &(p_right_last[8]), euler_last, translation_last);


                display_GUI->display_images(image[0], image[1], featuresLeftImage, featuresRightImage, p_left, p_right,image_color[0], image_color[1]);

// DETERMINE DATA ASSOCIATION


                for (int i = 0; i < 6; i++)
                {
                    last_t_split[i] = t_split[i];
                }

                double * t_splitL = curveMatcher[0].getMatchT( featuresLeftImage[0], &(last_image[0]), &(last_image_track[0]),&(image[0]),&(last_image_color[0]),&(image_color[0]),&(last_params[0]), &(params[0]), 0, &left_assoc_flag, reset_data_assoc);
                double * t_splitR = curveMatcher[1].getMatchT( featuresLeftImage[1], &(last_image[0]), &(last_image_track[0]),&(image[0]),&(last_image_color[0]),&(image_color[0]),&(last_params[0]), &(params[0]), 1, &right_assoc_flag, reset_data_assoc);

                for (int i = 0; i < 3; i++)
                {
                    t_split[2*i] = t_splitL[i];
                    t_split[2*i+1] = t_splitR[i];
                }

                //cout << "T SPLIT PTS\n";
                //cout << t_split[0] << " " << t_split[2] << " "<< t_split[4] << " " << t_split[1] << " "<< t_split[3] << " " << t_split[5] << endl;

                //Determine state based on data assoc parameter, but only if valid measurement
                if(valid_measurement && !first_time)
                {
                    if (t_splitL[0] < -0.5)
                    {
                        left_assoc_flag = ONLY_ADD_STATE;
                        curveMatcher[0].updateMapCurves();
                        partial_curve_observed[0] = false;
                    }
                    else if (t_splitL[0] < 0.05 && t_splitL[1] > 0.95 && t_splitL[2] < 0.05)
                    {
                        if (partial_curve_observed[0])
                        {
                            left_assoc_flag = ONLY_UPDATE_ONE_STATE_PREV;
                        }
                        else
                        {
                            left_assoc_flag = ONLY_UPDATE_ONE_STATE;
                        }

                    }
                    else if (t_splitL[0] > 0.95 && t_splitL[1] < 0.05 && t_splitL[2] > 0.95)
                    {
                        if (partial_curve_observed[0])
                        {
                            left_assoc_flag = ONLY_UPDATE_ONE_STATE;
                        }
                        else
                        {
                            left_assoc_flag = ONLY_ADD_STATE;
                        }

                        partial_curve_observed[0] = false;
                        curveMatcher[0].updateMapCurves();

                    }
                    else
                    {
                        if(partial_curve_observed[0])
                        {
                            left_assoc_flag = UPDATE_TWO_STATES;
                        }
                        else
                        {
                            left_assoc_flag = ADD_AND_UPDATE_STATES;
                        }
                        partial_curve_observed[0] = true;

                    }


                    if (t_splitR[0] < -0.5)
                    {
                        right_assoc_flag = ONLY_ADD_STATE;
                        curveMatcher[1].updateMapCurves();
                        partial_curve_observed[1] = false;
                    }
                    else if (t_splitR[0] < 0.05 && t_splitR[1] > 0.95 && t_splitR[2] < 0.05)
                    {
                        if (partial_curve_observed[1])
                        {
                            right_assoc_flag = ONLY_UPDATE_ONE_STATE_PREV;
                        }
                        else
                        {
                            right_assoc_flag = ONLY_UPDATE_ONE_STATE;
                        }

                    }
                    else if (t_splitR[0] > 0.95 && t_splitR[1] < 0.05 && t_splitR[2] > 0.95)
                    {
                        if (partial_curve_observed[1])
                        {
                            right_assoc_flag = ONLY_UPDATE_ONE_STATE;
                        }
                        else
                        {
                            right_assoc_flag = ONLY_ADD_STATE;
                        }

                        partial_curve_observed[1] = false;
                        curveMatcher[1].updateMapCurves();

                    }
                    else
                    {
                        if(partial_curve_observed[1])
                        {
                            right_assoc_flag = UPDATE_TWO_STATES;
                        }
                        else
                        {
                            right_assoc_flag = ADD_AND_UPDATE_STATES;
                        }
                        partial_curve_observed[1] = true;

                    }
                }
                else if (first_time)
                {
                    left_assoc_flag = ADD_FIRST_STATES;
                    right_assoc_flag = ADD_FIRST_STATES;
                    first_time = false;
                }





                //Plot features on images
                for( int i = 0; i < featuresLeftImage[LEFT]->rows/2; i++)
                {
                        cvCircle(image_color[LEFT],cvPoint(featuresLeftImage[LEFT]->data.db[2*i],featuresLeftImage[LEFT]->data.db[2*i+1]),1,CV_RGB(0,0,0));
                }
                for( int i = 0; i < featuresLeftImage[RIGHT]->rows/2; i++)
                {
                        cvCircle(image_color[LEFT],cvPoint(featuresLeftImage[RIGHT]->data.db[2*i],featuresLeftImage[RIGHT]->data.db[2*i+1]),1,CV_RGB(0,0,0));
                }
                for( int i = 0; i < featuresRightImage[LEFT]->rows/2; i++)
                {
                        cvCircle(image_color[RIGHT],cvPoint(featuresRightImage[LEFT]->data.db[2*i],featuresRightImage[LEFT]->data.db[2*i+1]),1,CV_RGB(0,0,0));
                }
                for( int i = 0; i < featuresRightImage[RIGHT]->rows/2; i++)
                {
                        cvCircle(image_color[RIGHT],cvPoint(featuresRightImage[RIGHT]->data.db[2*i],featuresRightImage[RIGHT]->data.db[2*i+1]),1,CV_RGB(0,0,0));
                }




                cvShowImage("Left",image_color[LEFT]);
                cvShowImage("Right",image_color[RIGHT]);
                cvShowImage("Last Left",last_image_color[LEFT]);
                cvShowImage("Last Right",last_image_color[RIGHT]);
                //cvWaitKey(10);


//Figure out from data assoc what to do
// Determine whether to just add states, or update, or both
                if (left_assoc_flag == ADD_FIRST_STATES && right_assoc_flag == ADD_FIRST_STATES && valid_measurement)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        z16->data.db[i] = params[2*i];
                        z16->data.db[i+4] = params[2*i+1];
                        z16->data.db[i+8] = params[2*(i+4)];
                        z16->data.db[i+12] = params[2*(i+4)+1];
                        z->data.db[i] = params[2*i];
                        z->data.db[i+4] = params[2*i+1];
                        z->data.db[i+8] = params[2*(i+4)];
                        z->data.db[i+12] = params[2*(i+4)+1];
                    }
                            z16->data.db[16] = height;
                            z16->data.db[17] = phi;
                            z16->data.db[18] = theta;
                    EKF->AddFirstStates(z16);
                    state_limits.push_back(1.0);
                    state_limits.push_back(1.0);
                    left_curve_nums.push_back(0);
                    right_curve_nums.push_back(1);
                    num_map_curves+=2;
                }

                //Check if the measurement is valid, otherwise, ignore
                else if (valid_measurement)
                {

                    int num_curves_to_update = 0;
                    curves_to_update.clear();
                    std::vector<CvMat *> correspondence_matrices;

                    CvMat * tempx = cvCreateMat(4,1,CV_64FC1);
                    CvMat * tempy = cvCreateMat(4,1,CV_64FC1);


                    //Depending on left and right assoc_flags, decide whether or not to split the measurement into two!
                    if (left_assoc_flag & (UPDATE_TWO_STATES|ADD_AND_UPDATE_STATES))
                    {
                        //Split left into two curves
                        for (int i = 0; i < 4; i++)
                        {
                            tempx->data.db[i] = params[2*i];
                            tempy->data.db[i] = params[2*i+1];
                        }

                        EKF->GetSplitMatrices(t_splitL[1],A1,A2);
                        cvMatMul(A1,tempx,z1);                    //Curve 1/2
                        cvMatMul(A1,tempy,z2);
                        cvMatMul(A2,tempx,z3);                    //Curve 3/4
                        cvMatMul(A2,tempy,z4);

                        for (int i = 0; i < 4; i++)
                        {
                            z_left->data.db[i] = z1->data.db[i];
                            z_left->data.db[i+4] = z2->data.db[i];
                            z_left->data.db[i+8] = z3->data.db[i];
                            z_left->data.db[i+12] = z4->data.db[i];
                        }
                        EKF->GetSplitMatrices(t_splitL[0],A1l,A2l);
                        EKF->GetSplitMatrices(t_splitL[2],B1l,B2l);
                    }
                    if (right_assoc_flag & (UPDATE_TWO_STATES|ADD_AND_UPDATE_STATES))
                    {
                        //Split RIGHT into two curves
                        for (int i = 0; i < 4; i++)
                        {
                            tempx->data.db[i] = params[2*i+8];
                            tempy->data.db[i] = params[2*i+9];
                        }

                        EKF->GetSplitMatrices(t_splitR[1],A1,A2);
                        cvMatMul(A1,tempx,z1);                    //Curve 1/2
                        cvMatMul(A1,tempy,z2);
                        cvMatMul(A2,tempx,z3);                    //Curve 3/4
                        cvMatMul(A2,tempy,z4);

                        for (int i = 0; i < 4; i++)
                        {
                            z_right->data.db[i] = z1->data.db[i];
                            z_right->data.db[i+4] = z2->data.db[i];
                            z_right->data.db[i+8] = z3->data.db[i];
                            z_right->data.db[i+12] = z4->data.db[i];
                        }

                        EKF->GetSplitMatrices(t_splitR[0],A1r,A2r);
                        EKF->GetSplitMatrices(t_splitR[2],B1r,B2r);
                    }


                    //UPDATE AND ADD LEFT CURVES
                    switch (left_assoc_flag)
                    {
                        case ONLY_ADD_STATE:
                        {
                            for (int i = 0; i < 4; i++)
                            {
                                z8->data.db[i] = params[2*i];
                                z8->data.db[i+4] = params[2*i+1];
                                z->data.db[i] = params[2*i];
                                z->data.db[i+4] = params[2*i+1];
                            }

                            EKF->AddNewCurve(z8,I4);
                            state_limits.push_back(1.0);
                            left_curve_nums.push_back(num_map_curves);
                            num_map_curves++;
                            break;
                        }
                        case ONLY_UPDATE_ONE_STATE:
                        {
                            for (int i = 0; i < 4; i++)
                            {
                                z->data.db[i+8*num_curves_to_update] = params[2*i];
                                z->data.db[i+4+8*num_curves_to_update] = params[2*i+1];
                            }

                            correspondence_matrices.push_back(I4);
                            state_limits.at(*(left_curve_nums.end()-1)) = 1.0;
                            curves_to_update.push_back(*(left_curve_nums.end()-1));
                            num_curves_to_update++;
                            break;
                        }
                        case ONLY_UPDATE_ONE_STATE_PREV:
                        {
                            for (int i = 0; i < 4; i++)
                            {
                                z->data.db[i+8*num_curves_to_update] = params[2*i];
                                z->data.db[i+4+8*num_curves_to_update] = params[2*i+1];
                            }

                            correspondence_matrices.push_back(I4);
                            state_limits.at(*(left_curve_nums.end()-2)) = 1.0;
                            curves_to_update.push_back(*(left_curve_nums.end()-2));
                            num_curves_to_update++;
                            break;
                        }
                        case ADD_AND_UPDATE_STATES:
                        {
                            //Add the first curve to 'update list'
                            for (int i = 0; i < 8; i++)
                            {
                                z->data.db[i+8*num_curves_to_update] = z_left->data.db[i];
                            }

                            correspondence_matrices.push_back(A2l);
                            state_limits.at(*(left_curve_nums.end()-1)) = t_splitL[2];
                            curves_to_update.push_back(*(left_curve_nums.end()-1));
                            num_curves_to_update++;


                            //Add second curve as new state
                            for (int i = 0; i < 8; i++)
                                z8->data.db[i] = z_left->data.db[i+8];
                            EKF->AddNewCurve(z8,B1l);
                            state_limits.push_back(t_splitL[2]);
                            left_curve_nums.push_back(num_map_curves);
                            num_map_curves++;

                            break;
                        }
                        case UPDATE_TWO_STATES:
                        {
                            for (int i = 0; i < 16; i++)
                            {
                                z->data.db[i+8*num_curves_to_update] = z_left->data.db[i];
                            }

                            correspondence_matrices.push_back(A2l);
                            correspondence_matrices.push_back(B1l);
                            curves_to_update.push_back(*(left_curve_nums.end()-2));
                            curves_to_update.push_back(*(left_curve_nums.end()-1));
                            state_limits.at(*(left_curve_nums.end()-2)) = 1.0;
                            state_limits.at(*(left_curve_nums.end()-1)) = t_splitL[2];
                            num_curves_to_update+=2;
                            break;
                        }
                        case DISCARD_MEASUREMENT:
                        {
                            break;
                        }
                    }


                    //UPDATE AND ADD RIGHT CURVES
                    switch (right_assoc_flag)
                    {
                        case ONLY_ADD_STATE:
                        {
                            for (int i = 0; i < 4; i++)
                            {
                                z8->data.db[i] = params[2*(i+4)];
                                z8->data.db[i+4] = params[2*(i+4)+1];
                                z->data.db[i+8] = params[2*(i+4)];
                                z->data.db[i+12] = params[2*(i+4)+1];
                            }

                            EKF->AddNewCurve(z8,I4);
                            state_limits.push_back(1.0);
                            right_curve_nums.push_back(num_map_curves);
                            num_map_curves++;
                            break;
                        }
                        case ONLY_UPDATE_ONE_STATE:
                        {
                            for (int i = 0; i < 4; i++)
                            {
                                z->data.db[i+8*num_curves_to_update] = params[2*(i+4)];
                                z->data.db[i+4+8*num_curves_to_update] = params[2*(i+4)+1];
                            }

                            correspondence_matrices.push_back(I4);
                            state_limits.at(*(right_curve_nums.end()-1)) = 1.0;
                            curves_to_update.push_back(*(right_curve_nums.end()-1));
                            num_curves_to_update++;
                            break;
                        }
                        case ONLY_UPDATE_ONE_STATE_PREV:
                        {
                            for (int i = 0; i < 4; i++)
                            {
                                z->data.db[i+8*num_curves_to_update] = params[2*(i+4)];
                                z->data.db[i+4+8*num_curves_to_update] = params[2*(i+4)+1];
                            }

                            correspondence_matrices.push_back(I4);
                            state_limits.at(*(right_curve_nums.end()-2)) = 1.0;
                            curves_to_update.push_back(*(right_curve_nums.end()-2));
                            num_curves_to_update++;
                            break;
                        }
                        case ADD_AND_UPDATE_STATES:
                        {                    
                            //Add the first curve to 'update list'
                            for (int i = 0; i < 8; i++)
                            {
                                z->data.db[i+8*num_curves_to_update] = z_right->data.db[i];
                            }

                            correspondence_matrices.push_back(A2r);
                            state_limits.at(*(right_curve_nums.end()-1)) = 1.0;
                            curves_to_update.push_back(*(right_curve_nums.end()-1));
                            num_curves_to_update++;


                            //Add second curve as new state
                            for (int i = 0; i < 8; i++)
                                z8->data.db[i] = z_right->data.db[i+8];
                            EKF->AddNewCurve(z8,B1r);
                            state_limits.push_back(t_splitR[2]);
                            right_curve_nums.push_back(num_map_curves);
                            num_map_curves++;

                            break;
                        }
                        case UPDATE_TWO_STATES:
                        {
                            for (int i = 0; i < 16; i++)
                            {
                                z->data.db[i+8*num_curves_to_update] = z_right->data.db[i];
                            }

                            correspondence_matrices.push_back(A2r);
                            correspondence_matrices.push_back(B1r);
                            curves_to_update.push_back(*(right_curve_nums.end()-2));
                            curves_to_update.push_back(*(right_curve_nums.end()-1));
                            state_limits.at(*(right_curve_nums.end()-2)) = 1.0;
                            state_limits.at(*(right_curve_nums.end()-1)) = t_splitR[2];
                            num_curves_to_update+=2;
                            break;
                        }
                        case DISCARD_MEASUREMENT:
                        {
                            break;
                        }
                    }


                    if (num_curves_to_update > 0)
                    {
                        z->data.db[num_curves_to_update*8] = height;
                        z->data.db[num_curves_to_update*8+1] = phi;
                        z->data.db[num_curves_to_update*8+2] = theta;
                        EKF->UpdateNCurvesAndPoints(z, num_curves_to_update, &(correspondence_matrices),&(curves_to_update),&point_meas_existing[0],&correspondences[0],n_pts_existing);
                    }

                }
                curves_updated_last.clear();
                    for (int i = 0; i < curves_to_update.size(); i++)
                        curves_updated_last.push_back(curves_to_update.at(i));
                    /*cout << "Left curve nums: ";
                    for (int i = 0; i < left_curve_nums.size(); i++)
                        cout << left_curve_nums.at(i) << " ";
                    cout << endl;
                    cout << "Right curve nums: ";
                    for (int i = 0; i < right_curve_nums.size(); i++)
                        cout << right_curve_nums.at(i) << " ";
                    cout << endl;
                    cout << "Curves to update: ";
                    for (int i = 0; i < curves_to_update.size(); i++)
                        cout << curves_to_update[i] << " ";
                    cout << endl;*/


            }
            else
                valid_measurement = false;
            
            //cout << edges_detected << " " << valid_measurement << endl;

            state_current = EKF->getState();
            //cout << "X:\n";
            //EKF->printMatrix(state_current);

            //Display curves
            display_GUI->generate_map(state_current,&state_limits,z, EKF->num_curves, EKF->num_points);
            //Print robot pose
            //cout << "X: " << state_current->data.db[0] << "\n" << "Y: " << state_current->data.db[1] << "\n" << "Z: " << state_current->data.db[2] << "\n"  << "Pitch: " << state_current->data.db[4]*180.0/PI << "\n" << "Roll: " << state_current->data.db[3]*180.0/PI << "\n" << "Yaw: " << state_current->data.db[5]*180.0/PI << "\n";
#endif


            int frameTime = camera[0].getTime()+camera[1].getTime();
            int featureTime = features[0].getTime()+features[1].getTime();
            int dataAssocTime = curveMatcher[0].getTime()+curveMatcher[1].getTime();
            int curveFitTime = curveFitter->getTime();
            int EKFTime = EKF->getTime();
            int displayTime = display_GUI->getTime();

            /*cout << "Frame grab: " << frameTime << " msec\t";
            cout << "Feature detection: " << featureTime << " msec\t";
            cout << "Data Assoc: " << dataAssocTime << " msec\t";
            cout << "Curve Fit: " << curveFitTime << " msec\t";
            cout << "EKF: " << EKFTime << " msec\t";
            cout << "Display: " << displayTime << " msec\t";*/

            EKF->resetTime();
            curveFitter->resetTime();
            display_GUI->resetTime();
            for (int i = 0; i < NUM_CAMERAS; ++i)
            {
                camera[i].resetTime();
                features[i].resetTime();
                curveMatcher[i].resetTime();
            }

            gettimeofday(&stop, NULL);
                elapsedTime = (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
                        (start.tv_sec*1000.0 + start.tv_usec/1000.0);
                        //cout << "TOTAL: " << elapsedTime << endl;
            gettimeofday(&start, NULL);

            //cvShowImage("Left",image_color[LEFT]);
            //cvShowImage("Right",image_color[RIGHT]);
            cvWaitKey(0);
        }

	return 0;
}

