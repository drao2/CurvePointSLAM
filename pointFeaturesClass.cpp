#include "pointFeaturesClass.h"
#include "common.h"


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

void PointFeaturesClass::getPointMeasurements(IplImage ** last_image, IplImage ** image, IplImage ** image_color, double * meas_new, int * num_new_meas, double * meas_existing, int * num_existing_meas, int * correspondences, double * planar_pose_meas)
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
        
        double stereo_meas_existing[200];
        double stereo_meas_new[200];
        
        //Track existing points and Add new ones to make up a constant number of total tracked points
        trackExistingLandmarks(last_image, image, &stereo_meas_existing[0], num_existing_meas, correspondences);
        findNewLandmarks(image, &stereo_meas_new[0], num_new_meas);
        //getStereoPts(image, stereo_pts, &num_total_meas);
        //determineDataAssoc(image, stereo_pts, num_total_meas, meas_new, num_new_meas, meas_existing, num_existing_meas, correspondences);
        

        
        //
        //Triangulate all points, and fit a plane to get the 2D landmarks and the roll, pitch, height
        //
        int num_tot_meas = *num_existing_meas + *num_new_meas;
        
        //Matrices required
        /*centroid = cvCreateMat(3,1,CV_64FC1);
        CvMat * W = cvCreateMat(3,1, CV_64FC1);
        CvMat * U = cvCreateMat(num_tot_meas,3, CV_64FC1);
        CvMat * V = cvCreateMat(3,3, CV_64FC1);
        CvMat * normal = cvCreateMat(3,1,CV_64FC1);
        CvMat * temp13 = cvCreateMat(1,3,CV_64FC1);
        CvMat * centroid = cvCreateMat(3,1,CV_64FC1);
        CvMat * origin = cvCreateMat(3,1,CV_64FC1);
        CvMat * Rot = cvCreateMat(3,3,CV_64FC1);
        
        //First, put triangulated measurements into a matrix (to pass into SVD)
        CvMat * M = cvCreateMat(num_tot_meas,3,CV_64FC1);
        CvMat * proj_pts = cvCreateMat(3,num_tot_meas,CV_64FC1);
        cvSetZero(M);
        cvSetZero(proj_pts);
        
        
        for (int i = 0; i < *num_existing_meas; i++)
        {
            double x = FX*BASELINE/(stereo_meas_existing[4*i]-stereo_meas_existing[4*i+2]);
            cvmSet(M,i,0, x);
            cvmSet(M,i,1, 0.5*(x/FX*(stereo_meas_existing[4*i]-CX)-0.5*BASELINE)+0.5*(x/FX*(stereo_meas_existing[4*i+2]-CX)+0.5*BASELINE));
            cvmSet(M,i,2, 0.5*x/FY*(stereo_meas_existing[4*i+1]-CY)+0.5*x/FY*(stereo_meas_existing[4*i+3]-CY));
        }
        for (int i = *num_existing_meas; i < num_tot_meas; i++)
        {
            double x = FX*BASELINE/(stereo_meas_new[4*i]-stereo_meas_new[4*i+2]);
            cvmSet(M,i,0, x);
            cvmSet(M,i,1, 0.5*(x/FX*(stereo_meas_new[4*i]-CX)-0.5*BASELINE)+0.5*(x/FX*(stereo_meas_new[4*i+2]-CX)+0.5*BASELINE));
            cvmSet(M,i,2, 0.5*x/FY*(stereo_meas_new[4*i+1]-CY)+0.5*x/FY*(stereo_meas_new[4*i+3]-CY));
        }
        printMatrix(M);
        
        //Get the centroid of the 3D points
        //Just sum over columns of M
        cvReduce(M, temp13, 0, CV_REDUCE_SUM);
        centroid->data.db[0] = temp13->data.db[0]/num_tot_meas;
        centroid->data.db[1] = temp13->data.db[1]/num_tot_meas;
        centroid->data.db[2] = temp13->data.db[2]/num_tot_meas;
        printMatrix(centroid);
        
        //Find the normal vector to the plane they lie in
        cvSVD(M, W, U, V, 0);

        normal->data.db[0] = V->data.db[2];
        normal->data.db[1] = V->data.db[5];
        normal->data.db[2] = V->data.db[8];

        double d = cvDotProduct(normal,centroid);
        double theta = -asin(normal->data.db[0]);
        double phi = asin(normal->data.db[1]);

        //Find the point of the 'ground' co-ord axis
        origin->data.db[0] = d*normal->data.db[0];
        origin->data.db[1] = d*normal->data.db[1];
        origin->data.db[2] = d*normal->data.db[2];
        

        double orignorm = cvNorm(origin,NULL,CV_L2,NULL);
        
        Rot->data.db[0] = cos(theta);
        Rot->data.db[3] = 0.0;
        Rot->data.db[6] = -sin(theta);
        Rot->data.db[1] = sin(theta)*sin(phi);
        Rot->data.db[4] = cos(phi);
        Rot->data.db[7] = cos(theta)*sin(phi);
        Rot->data.db[2] = sin(theta)*cos(phi);
        Rot->data.db[5] = -sin(phi);
        Rot->data.db[8] = cos(theta)*cos(phi);

        //Transform points by projecting onto plane
        cvTranspose(M, proj_pts);
        printMatrix(proj_pts);
                //Subtract origin from each point first
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < num_tot_meas; j++)
                proj_pts->data.db[i*num_tot_meas+j] -= origin->data.db[i];
        }
        
                //Then rotate to new co-ords (should be close to 2D)
        cvMatMul(Rot,proj_pts,proj_pts);
        printMatrix(proj_pts);
        
        */
        
        for (int i = 0; i < *num_existing_meas; i++)
        {
            double x = FX*BASELINE/(stereo_meas_existing[4*i]-stereo_meas_existing[4*i+2]);
            meas_existing[3*i] = x;
            meas_existing[3*i+1] = 0.5*(x/FX*(stereo_meas_existing[4*i]-CX)-0.5*BASELINE)+0.5*(x/FX*(stereo_meas_existing[4*i+2]-CX)+0.5*BASELINE);
            meas_existing[3*i+2] = 0.5*x/FY*(stereo_meas_existing[4*i+1]-CY)+0.5*x/FY*(stereo_meas_existing[4*i+3]-CY);
        }
        for (int i = 0; i < *num_new_meas; i++)
        {
            double x = FX*BASELINE/(stereo_meas_new[4*i]-stereo_meas_new[4*i+2]);
            meas_new[3*i] = x;
            meas_new[3*i+1] = 0.5*(x/FX*(stereo_meas_new[4*i]-CX)-0.5*BASELINE)+0.5*(x/FX*(stereo_meas_new[4*i+2]-CX)+0.5*BASELINE);
            meas_new[3*i+2] = 0.5*x/FY*(stereo_meas_new[4*i+1]-CY)+0.5*x/FY*(stereo_meas_new[4*i+3]-CY);
        }
        
        
        
        
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
        cvCircle(image_color[LEFT], cvPoint(stereo_meas_existing[4*i],stereo_meas_existing[4*i+1]), 5, CV_RGB(255,255,255), 1, CV_AA, 0 );
        sprintf(tstring,"%d",correspondences[i]);
        cvPutText(image_color[LEFT],tstring,cvPoint(stereo_meas_existing[4*i],stereo_meas_existing[4*i+1]-5),&font, CV_RGB(255,255,255));
        
        cvCircle(image_color[RIGHT], cvPoint(stereo_meas_existing[4*i+2],stereo_meas_existing[4*i+3]), 5, CV_RGB(255,255,255), 1, CV_AA, 0 );
        sprintf(tstring,"%d",correspondences[i]);
        cvPutText(image_color[RIGHT],tstring,cvPoint(stereo_meas_existing[4*i+2],stereo_meas_existing[4*i+3]-5),&font, CV_RGB(255,255,255));
    }
    for (int i = 0; i < *num_new_meas; i++)
    {
        cvCircle(image_color[LEFT], cvPoint(stereo_meas_new[4*i],stereo_meas_new[4*i+1]), 5, CV_RGB(0,0,0), 1, CV_AA, 0 );
        sprintf(tstring,"%d",-1);
        cvPutText(image_color[LEFT],tstring,cvPoint(stereo_meas_new[4*i],stereo_meas_new[4*i+1]-5),&font, CV_RGB(0,0,0));
        
        cvCircle(image_color[RIGHT], cvPoint(stereo_meas_new[4*i+2],stereo_meas_new[4*i+3]), 5, CV_RGB(0,0,0), 1, CV_AA, 0 );
        sprintf(tstring,"%d",-1);
        cvPutText(image_color[RIGHT],tstring,cvPoint(stereo_meas_new[4*i+2],stereo_meas_new[4*i+3]-5),&font, CV_RGB(0,0,0));
    }
    //cvShowImage("Left",image_color[LEFT]);
    //cvShowImage("Right",image_color[RIGHT]);
    //cvWaitKey(0);
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
                    //If disparity OK (and not too different to last time)
                    if((temp_pts_curr[LEFT][i].x-temp_pts_curr[RIGHT][i].x) < DISP_THRESH_HI && (temp_pts_curr[LEFT][i].x-temp_pts_curr[RIGHT][i].x) > DISP_THRESH_LO && ( fabs(temp_pts_curr[LEFT][i].x-temp_pts_curr[RIGHT][i].x - stereo_disparity[i]) < DISP_THRESH_CHANGE ) )
                    {
                        //If not too close to the edge
                        if(temp_pts_curr[LEFT][i].x > PATCH_HALF+BORDER_LEFT && temp_pts_curr[LEFT][i].x < BORDER_RIGHT-PATCH_HALF-0 && temp_pts_curr[LEFT][i].y > PATCH_HALF+BORDER_TOP && temp_pts_curr[LEFT][i].y < BORDER_BOTTOM-PATCH_HALF)
                        {
                            if(temp_pts_curr[RIGHT][i].x > PATCH_HALF+BORDER_LEFT && temp_pts_curr[RIGHT][i].x < BORDER_RIGHT-PATCH_HALF-0 && temp_pts_curr[RIGHT][i].y > PATCH_HALF+BORDER_TOP && temp_pts_curr[RIGHT][i].y < BORDER_BOTTOM-PATCH_HALF)
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
                                    
                                    //Maintain stereo disparity (so we can check for next time)
                                    stereo_disparity[num_pts_curr] = temp_pts_curr[LEFT][i].x - temp_pts_curr[RIGHT][i].x;

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
        cvGoodFeaturesToTrack(image[LEFT], eig_image, temp_image, image_features_left, &num_features_left, 0.0001, 5, NULL, 3, 0, 0.04);
        //Find right image features
        CvPoint2D32f image_features_right[MAX_VODOM_FEATURES];
        int num_features_right = MAX_VODOM_FEATURES;
        cvGoodFeaturesToTrack(image[RIGHT], eig_image, temp_image, image_features_right, &num_features_right, 0.0001, 5, NULL, 3, 0, 0.04);

        //Remove ones that are too close to the edges
        for (int i = 0; i < num_features_left; i++)
        {
            while (image_features_left[i].x <= WINDOW_HALF+BORDER_LEFT || image_features_left[i].x >= BORDER_RIGHT-WINDOW_HALF || image_features_left[i].y <= BORDER_TOP+WINDOW_HALF || image_features_left[i].y >= BORDER_BOTTOM-WINDOW_HALF )
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

            //Only keep ones that share mutual matches and whose disparity
            //means the distance from camera is close enough

            CvPoint2D32f matched_features_left[MAX_VODOM_FEATURES];
            CvPoint2D32f matched_features_right[MAX_VODOM_FEATURES];
            int k = 0;

            for (int i = 0; i < num_features_left; i++)
            {
                if (left_bestmatch[i] != -1 && right_bestmatch[left_bestmatch[i]] == i && (image_features_left[i].x - image_features_right[left_bestmatch[i]].x) > FX*BASELINE/MAX_PT_DIST)
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
            // TODO: SORT BY HOW GOOD THEY ARE!!!!
            for (int i = 0; i < num_features_to_add; i++)
            {
                pts_curr[LEFT][num_pts_curr] = matched_features_left[i];
                pts_curr[RIGHT][num_pts_curr] = matched_features_right[i];
                
                stereo_disparity[num_pts_curr] = pts_curr[LEFT][i].x - pts_curr[RIGHT][i].x;


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
