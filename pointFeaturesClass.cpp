#include "pointFeaturesClass.h"
#include "common.h"


//TODO: FIX UP CALIBRATION STUFF - the body2stereo and reverse don't reproduce the same thing
//TODO: FIX UP RANSAC COST STUFF - could only use 2 points, get a horribly wrong estimate but a good cost

using namespace std;

PointFeaturesClass::PointFeaturesClass()
{
        CvSize frame_size = cvSize(PIC_WIDTH,PIC_HEIGHT);

        eig_image = cvCreateImage( frame_size, IPL_DEPTH_32F, 1 );
        temp_image = cvCreateImage( frame_size, IPL_DEPTH_32F, 1 );
        pyramid1 = cvCreateImage(cvSize(PIC_WIDTH+8,PIC_HEIGHT/3), IPL_DEPTH_8U, 1);
        pyramid2 = cvCreateImage(cvSize(PIC_WIDTH+8,PIC_HEIGHT/3), IPL_DEPTH_8U, 1);
        //srand(0);
	//return;
        
        optical_flow_window = cvSize(31,31);
        optical_flow_termination_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, .0001 );
        
        num_pts_last = 0;
        num_pts_curr = 0;
        
        num_landmarks_total = 0;
}

PointFeaturesClass::~PointFeaturesClass()
{
        
        //optical_flow_window = cvSize(25,25);
        //optical_flow_termination_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, .01 );
	//return;
}

void PointFeaturesClass::getPointMeasurements(IplImage ** last_image, IplImage ** image, IplImage ** image_color, double * meas_new, int * num_new_meas, double * meas_existing, int * num_existing_meas, int * correspondences)
{ 
    
timeval t_start, t_stop;
timeval start, stop;
float elapsedTime;
        for (int i = 0; i < existing_landmarks.size(); i++)
            existing_landmarks[i].frames_since_obs++;

        //First, copy 'current' to 'last' landmarks observed
        for (int i = 0; i < num_pts_curr; i++)
        {
            pts_last[LEFT][i] = pts_curr[LEFT][i];
            pts_last[RIGHT][i] = pts_curr[RIGHT][i];
            matches_last[i] = matches_curr[i];
        }
        num_pts_last = num_pts_curr;
        num_pts_curr = 0;
        gettimeofday(&start, NULL);
        trackExistingLandmarks(last_image, image, meas_existing, num_existing_meas, correspondences);
        findNewLandmarks(image, meas_new, num_new_meas);
        //getStereoPts(image, stereo_pts, &num_total_meas);
        //determineDataAssoc(image, stereo_pts, num_total_meas, meas_new, num_new_meas, meas_existing, num_existing_meas, correspondences);
        
        
        
        
        
        gettimeofday(&stop, NULL);
        
	elapsedTime = (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
        
        //cout << "Params: ";
        //for (int i = 0; i < 6; i++)
        //    cout << params[i] << " ";
        //cout << endl;
        //cout << "Elapsed Time: " << elapsedTime << endl;
        
                //Plots to debug
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX , 0.5f, 0.5f, 0, 1, 8);
    char tstring[5];
    for (int i = 0; i < *num_existing_meas; i++)
    {
        cvCircle(image_color[LEFT], cvPoint(meas_existing[4*i],meas_existing[4*i+1]), 5, CV_RGB(255,255,255), 1, CV_AA, 0 );
        sprintf(tstring,"%d",correspondences[i]);
        cvPutText(image_color[LEFT],tstring,cvPoint(meas_existing[4*i],meas_existing[4*i+1]-5),&font, CV_RGB(255,255,255));
        
        cvCircle(image_color[RIGHT], cvPoint(meas_existing[4*i+2],meas_existing[4*i+3]), 5, CV_RGB(255,255,255), 1, CV_AA, 0 );
        sprintf(tstring,"%d",correspondences[i]);
        cvPutText(image_color[RIGHT],tstring,cvPoint(meas_existing[4*i+2],meas_existing[4*i+3]-5),&font, CV_RGB(255,255,255));
    }
    for (int i = 0; i < *num_new_meas; i++)
    {
        cvCircle(image_color[LEFT], cvPoint(meas_new[4*i],meas_new[4*i+1]), 5, CV_RGB(0,0,0), 1, CV_AA, 0 );
        sprintf(tstring,"%d",-1);
        cvPutText(image_color[LEFT],tstring,cvPoint(meas_new[4*i],meas_new[4*i+1]-5),&font, CV_RGB(0,0,0));
        
        cvCircle(image_color[RIGHT], cvPoint(meas_new[4*i+2],meas_new[4*i+3]), 5, CV_RGB(0,0,0), 1, CV_AA, 0 );
        sprintf(tstring,"%d",-1);
        cvPutText(image_color[RIGHT],tstring,cvPoint(meas_new[4*i+2],meas_new[4*i+3]-5),&font, CV_RGB(0,0,0));
    }
    cvShowImage("Left",image_color[LEFT]);
    cvShowImage("Right",image_color[RIGHT]);
    cvWaitKey(10);
}


void PointFeaturesClass::trackExistingLandmarks(IplImage ** last_image, IplImage ** image, double * meas_existing, int *num_existing_meas, int * correspondence)
{
    //Only bother tracking if we have any previous viewed landmarks
    if(num_pts_last)
    {
        //Track left and right points to current image
        cvCalcOpticalFlowPyrLK(last_image[LEFT], image[LEFT], pyramid1, pyramid2, &(pts_last[LEFT][0]),&(temp_pts_curr[LEFT][0]), num_pts_last, optical_flow_window, 3,optical_flow_found_feature_left, optical_flow_feature_error_left,optical_flow_termination_criteria, 0 );
        cvCalcOpticalFlowPyrLK(last_image[RIGHT], image[RIGHT], pyramid1, pyramid2, &(pts_last[RIGHT][0]),&(temp_pts_curr[RIGHT][0]), num_pts_last, optical_flow_window, 3,optical_flow_found_feature_right, optical_flow_feature_error_right,optical_flow_termination_criteria, 0 );
        
        num_pts_curr = 0;
        *num_existing_meas = 0;
        //Check tracked points
        //Remove any that don't seem right (disparity), or are bad tracks (!feature_found), or have SSD above a thresh
        //Copy the rest over to the pts_curr vector and also to meas_existing vector
        for (int i = 0; i < num_pts_last; i++)
        {
            //If tracked OK
            if(optical_flow_found_feature_left[i] && optical_flow_found_feature_right[i])
            {
                
                //If close enough in terms of epipole
                if(fabs(temp_pts_curr[LEFT][i].y-temp_pts_curr[RIGHT][i].y) < EPI_THRESH)
                {
                    //If disparity OK
                    if((temp_pts_curr[LEFT][i].x-temp_pts_curr[RIGHT][i].x) > 0.0 && (temp_pts_curr[LEFT][i].x-temp_pts_curr[RIGHT][i].x) < 50.0)
                    {
                        //If not too close to the edge
                        if(temp_pts_curr[LEFT][i].x > PATCH_HALF+0 && temp_pts_curr[LEFT][i].x < PIC_WIDTH-PATCH_HALF-0 && temp_pts_curr[LEFT][i].y > PATCH_HALF+0 && temp_pts_curr[LEFT][i].y < PIC_HEIGHT-PATCH_HALF-0)
                        {
                            if(temp_pts_curr[RIGHT][i].x > PATCH_HALF+0 && temp_pts_curr[RIGHT][i].x < PIC_WIDTH-PATCH_HALF-0 && temp_pts_curr[RIGHT][i].y > PATCH_HALF+0 && temp_pts_curr[RIGHT][i].y < PIC_HEIGHT-PATCH_HALF-0)
                            {
                                //If SSD matches reasonably between tracked features for both images
                                cvSetImageROI(image[LEFT],cvRect(temp_pts_curr[LEFT][i].x-PATCH_HALF,temp_pts_curr[LEFT][i].y-PATCH_HALF,PATCH_SIZE,PATCH_SIZE));
                                cvSetImageROI(image[RIGHT],cvRect(temp_pts_curr[RIGHT][i].x-PATCH_HALF,temp_pts_curr[RIGHT][i].y-PATCH_HALF,PATCH_SIZE,PATCH_SIZE));

                                //cout << image[LEFT]->roi->width << " " << image[LEFT]->roi->height << endl;
                                //cout << image[RIGHT]->roi->width << " " << image[RIGHT]->roi->height << endl << endl;

                                if(cvNorm(image[LEFT],image[RIGHT]) < SSD_THRESH)
                                {
                                    pts_curr[LEFT][num_pts_curr] = temp_pts_curr[LEFT][i];
                                    pts_curr[RIGHT][num_pts_curr] = temp_pts_curr[RIGHT][i];

                                    meas_existing[4*num_pts_curr] = temp_pts_curr[LEFT][i].x;
                                    meas_existing[4*num_pts_curr+1] = temp_pts_curr[LEFT][i].y;
                                    meas_existing[4*num_pts_curr+2] = temp_pts_curr[RIGHT][i].x;
                                    meas_existing[4*num_pts_curr+3] = temp_pts_curr[RIGHT][i].y;
                                    matches_curr[num_pts_curr] = matches_last[i];
                                    correspondence[num_pts_curr] = matches_last[i];
                                    //cout << "Feature " << correspondence[num_pts_curr] << ": Left error " << optical_flow_feature_error_left[i] << ", Right Error " << optical_flow_feature_error_right[i] << endl;
                                    num_pts_curr++;
                                }
                                cvResetImageROI(image[LEFT]);
                                cvResetImageROI(image[RIGHT]);
                            }
                        }
                    }
                }
            }
        }
        *num_existing_meas = num_pts_curr;
    }
    
}

void PointFeaturesClass::findNewLandmarks(IplImage ** image, double * meas_new, int *num_new_meas)
{
    //Only bother if we don't have the maximum allowed # landmarks already
    if (num_pts_curr < NUM_CURRENT_LANDMARKS)
    {
        //Find left image features
        CvPoint2D32f image_features_left[MAX_VODOM_FEATURES];
        int num_features_left = MAX_VODOM_FEATURES;
        cvGoodFeaturesToTrack(image[LEFT], eig_image, temp_image, image_features_left, &num_features_left, 0.01, 20, NULL, 13, 0, 0.04);
        //Find right image features
        CvPoint2D32f image_features_right[MAX_VODOM_FEATURES];
        int num_features_right = MAX_VODOM_FEATURES;
        cvGoodFeaturesToTrack(image[RIGHT], eig_image, temp_image, image_features_right, &num_features_right, 0.01, 20, NULL, 13, 0, 0.04);

        //Remove ones that are too close to the edges
        for (int i = 0; i < num_features_left; i++)
        {
            while (image_features_left[i].x <= WINDOW_HALF || image_features_left[i].x >= PIC_WIDTH-WINDOW_HALF || image_features_left[i].y <= 50+WINDOW_HALF || image_features_left[i].y >= PIC_HEIGHT-WINDOW_HALF )
            {
                image_features_left[i] =  image_features_left[num_features_left-1];
                num_features_left--;
                if (num_features_left <= 0)
                    break;
            }
        }
        for (int i = 0; i < num_features_right; i++)
        {
            while (image_features_right[i].x <= WINDOW_HALF || image_features_right[i].x >= PIC_WIDTH-WINDOW_HALF || image_features_right[i].y <= 50+WINDOW_HALF || image_features_right[i].y >= PIC_HEIGHT-WINDOW_HALF )
            {
                image_features_right[i] =  image_features_right[num_features_right-1];
                num_features_right--;
                if (num_features_right <= 0)
                    break;
            }
        }
        
        //Remove ones that are sufficiently close to the ones we have already
        for (int j = 0; j < num_pts_curr; j++)
        {
            for (int i = 0; i < num_features_left; i++)
            {
                while (DIST(image_features_left[i],pts_curr[LEFT][j]) < LM_PROXIMITY_THRESH )
                {
                    image_features_left[i] =  image_features_left[num_features_left-1];
                    num_features_left--;
                    if (num_features_left <= 0)
                        break;
                }
            }
            for (int i = 0; i < num_features_right; i++)
            {
                while (DIST(image_features_right[i],pts_curr[RIGHT][j]) < LM_PROXIMITY_THRESH )
                {
                    image_features_right[i] =  image_features_right[num_features_right-1];
                    num_features_right--;
                    if (num_features_right <= 0)
                        break;
                }
            }
        }


        if (num_features_left != 0 && num_features_right != 0)
        {
            //Find stereo matches (compute SSDs and construct score matrix)
            //cout << num_last_features_left << " " << num_last_features_right << endl;
            CvMat * score = cvCreateMat(num_features_left,num_features_right,CV_32F);
            int left_bestmatch[num_features_left];
            int right_bestmatch[num_features_right];
            for (int i = 0; i < num_features_left; i++)
            {
                for (int j = 0; j < num_features_right; j++)
                {
                    CvPoint left_feat = cvPoint(image_features_left[i].x,image_features_left[i].y);
                    CvPoint right_feat = cvPoint(image_features_right[j].x,image_features_right[j].y);

                    if (abs(right_feat.y-left_feat.y) > EPI_THRESH || left_feat.x-right_feat.x >= DISP_THRESH_HI || left_feat.x-right_feat.x <= DISP_THRESH_LO)
                        cvmSet(score,i,j,1000000.0);
                    else
                    {
                        cvSetImageROI(image[LEFT],cvRect(left_feat.x-WINDOW_HALF,left_feat.y-WINDOW_HALF,WINDOW_SIZE,WINDOW_SIZE));
                        cvSetImageROI(image[RIGHT],cvRect(right_feat.x-WINDOW_HALF,right_feat.y-WINDOW_HALF,WINDOW_SIZE,WINDOW_SIZE));
                        //cout << left_feat.x << " " << left_feat.y << endl;
                        //cout << right_feat.x << " " << right_feat.y << endl;
                        cvmSet(score,i,j,cvNorm(image[LEFT],image[RIGHT]));

                        cvResetImageROI(image[LEFT]);
                        cvResetImageROI(image[RIGHT]);
                    }
                }
            }

            //Find left_bestmatches
            for (int i = 0; i < num_features_left; i++)
            {
                left_bestmatch[i] = -1;
                double current_best_cost = SSD_THRESH;
                for (int j = 0; j < num_features_right; j++)
                {
                    CvPoint left_feat = cvPoint(image_features_left[i].x,image_features_left[i].y);
                    CvPoint right_feat = cvPoint(image_features_right[j].x,image_features_right[j].y);

                    if (abs(right_feat.y-left_feat.y) <= EPI_THRESH)
                    {
                        double ssd = cvmGet(score,i,j);
                        if(ssd < current_best_cost)
                        {
                            current_best_cost = ssd;
                            left_bestmatch[i] = j;
                        }
                    }
                }
            }
            //Find right_bestmatches
            for (int j = 0; j < num_features_right; j++)
            {
                right_bestmatch[j] = -1;
                double current_best_cost = SSD_THRESH;
                for (int i = 0; i < num_features_left; i++)
                {
                    CvPoint left_feat = cvPoint(image_features_left[i].x,image_features_left[i].y);
                    CvPoint right_feat = cvPoint(image_features_right[j].x,image_features_right[j].y);

                    if (abs(right_feat.y-left_feat.y) <= EPI_THRESH)
                    {
                        double ssd = cvmGet(score,i,j);
                        if(ssd < current_best_cost)
                        {
                            current_best_cost = ssd;
                            right_bestmatch[j] = i;
                        }
                    }
                }
            }

            //Only keep ones that share mutual matches and whose disparity is >0

            CvPoint2D32f matched_features_left[MAX_VODOM_FEATURES];
            CvPoint2D32f matched_features_right[MAX_VODOM_FEATURES];
            int k = 0;

            for (int i = 0; i < num_features_left; i++)
            {
                if (left_bestmatch[i] != -1 && right_bestmatch[left_bestmatch[i]] == i && (image_features_left[i].x - image_features_right[left_bestmatch[i]].x) > -2*POINT_OFFSET)
                {
                    matched_features_left[k] =  image_features_left[i];
                    matched_features_right[k] =  image_features_right[left_bestmatch[i]];
                    k++;
                }
            }

            int num_features_to_add = MIN(k,NUM_CURRENT_LANDMARKS - num_pts_curr);
            int n_new_pts = 0;
            
            //Copy whatever we need (only to make up the rest) to pts_curr and meas_new
            //Also update matches_curr
            for (int i = 0; i < num_features_to_add; i++)
            {
                pts_curr[LEFT][num_pts_curr] = matched_features_left[i];
                pts_curr[RIGHT][num_pts_curr] = matched_features_right[i];

                meas_new[4*n_new_pts] = matched_features_left[i].x;
                meas_new[4*n_new_pts+1] = matched_features_left[i].y;
                meas_new[4*n_new_pts+2] = matched_features_right[i].x;
                meas_new[4*n_new_pts+3] = matched_features_right[i].y;
                
                matches_curr[num_pts_curr] = num_landmarks_total;
                num_landmarks_total++;
                num_pts_curr++;
                n_new_pts++;
            } 
            *num_new_meas = n_new_pts;
            //cout << "Num stereo matched features: " << num_matched_features << endl;
        }
    }
}
