#include "vodom.h"
#include "common.h"


//TODO: FIX UP CALIBRATION STUFF - the body2stereo and reverse don't reproduce the same thing
//TODO: FIX UP RANSAC COST STUFF - could only use 2 points, get a horribly wrong estimate but a good cost

using namespace std;

VisualOdomClass::VisualOdomClass()
{
          CvSize frame_size = cvSize(PIC_WIDTH,PIC_HEIGHT);

        eig_image = cvCreateImage( frame_size, IPL_DEPTH_32F, 1 );
        temp_image = cvCreateImage( frame_size, IPL_DEPTH_32F, 1 );
        pyramid1 = cvCreateImage(cvSize(PIC_WIDTH+8,PIC_HEIGHT/3), IPL_DEPTH_8U, 1);
        pyramid2 = cvCreateImage(cvSize(PIC_WIDTH+8,PIC_HEIGHT/3), IPL_DEPTH_8U, 1);
        srand(0);
	return;
}

VisualOdomClass::~VisualOdomClass()
{
        
        optical_flow_window = cvSize(25,25);
        optical_flow_termination_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, .01 );
	return;
}

void VisualOdomClass::getPoseEstimate(IplImage ** last_image, IplImage ** current_image, double * params)
{ 
    
timeval t_start, t_stop;
timeval start, stop;
float elapsedTime;


        //Get stereo feature pairs, matched, in last and current images
        double last_stereo_pts[MAX_VODOM_FEATURES*4];
        double current_stereo_pts[MAX_VODOM_FEATURES*4];
        int num_features = 3;
        
        //double last_stereo_pts[] = {170.0,100.0,150.0,100.0,170.0,120.0,150.0,120.0,170.0,140.0,150.0,140.0};
        //double current_stereo_pts[] = {170.0,100.0,150.0,100.0,170.0,120.0,150.0,120.0,170.0,140.0,150.0,140.0};
        //double current_stereo_pts[] = {175.0,95.0,145.0,95.0,175.0,120.0,145.0,120.0,175.0,145.0,145.0,145.0};
        
        
        
        gettimeofday(&start, NULL);
        getStereoPts(last_image, current_image, last_stereo_pts, current_stereo_pts, &num_features);
        /*cout << "Last pts:\n";
        for (int i = 0; i < num_features; i++)
        {
            cout << i << ": " << last_stereo_pts[4*i] << " " << last_stereo_pts[4*i+1] << " "  << last_stereo_pts[4*i+2] << " "  << last_stereo_pts[4*i+3] << endl; 
        }
        cout << "Current pts:\n";
        for (int i = 0; i < num_features; i++)
        {
            cout << i << ": " << current_stereo_pts[4*i] << " " << current_stereo_pts[4*i+1] << " "  << current_stereo_pts[4*i+2] << " "  << current_stereo_pts[4*i+3] << endl; 
        }*/
        if(num_features)
        {
            PoseFromRANSAC(&(last_stereo_pts[0]), &(current_stereo_pts[0]), num_features, &(params[0]));
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
    for (int i = 0; i < num_features; i++)
    {
        //cvCircle(last_image[LEFT], cvPoint(last_stereo_pts[4*i],last_stereo_pts[4*i+1]), 5, CV_RGB(255,255,255), 1, CV_AA, 0 );
        //cvCircle(current_image[LEFT], cvPoint(current_stereo_pts[4*i],current_stereo_pts[4*i+1]), 5, CV_RGB(255,255,255), 1, CV_AA, 0 );
        sprintf(tstring,"%d",i);
        //cvPutText(last_image[LEFT],tstring,cvPoint(last_stereo_pts[4*i],last_stereo_pts[4*i+1]-5),&font, CV_RGB(255,255,255));
        //cvPutText(current_image[LEFT],tstring,cvPoint(current_stereo_pts[4*i],current_stereo_pts[4*i+1]-5),&font, CV_RGB(255,255,255));
        
        //cvCircle(last_image[RIGHT], cvPoint(last_stereo_pts[4*i+2],last_stereo_pts[4*i+3]), 5, CV_RGB(255,255,255), 1, CV_AA, 0 );
        //cvCircle(current_image[RIGHT], cvPoint(current_stereo_pts[4*i+2],current_stereo_pts[4*i+3]), 5, CV_RGB(255,255,255), 1, CV_AA, 0 );
        sprintf(tstring,"%d",i);
        //cvPutText(last_image[RIGHT],tstring,cvPoint(last_stereo_pts[4*i+2],last_stereo_pts[4*i+3]-5),&font, CV_RGB(255,255,255));
        //cvPutText(current_image[RIGHT],tstring,cvPoint(current_stereo_pts[4*i+2],current_stereo_pts[4*i+3]-5),&font, CV_RGB(255,255,255));

    }
}

void VisualOdomClass::getStereoPts(IplImage ** last_image, IplImage ** current_image, double * last_stereo_pts, double * current_stereo_pts, int * num_features)
{
    //Find last_image left image features
    CvPoint2D32f last_image_features_left[MAX_VODOM_FEATURES];
    int num_last_features_left = MAX_VODOM_FEATURES;
    cvGoodFeaturesToTrack(last_image[LEFT], eig_image, temp_image, last_image_features_left, &num_last_features_left, 0.01, 20, NULL, 3, 0, 0.04);
    
    
    //Find last_image right image features
    CvPoint2D32f last_image_features_right[MAX_VODOM_FEATURES];
    int num_last_features_right = MAX_VODOM_FEATURES;
    cvGoodFeaturesToTrack(last_image[RIGHT], eig_image, temp_image, last_image_features_right, &num_last_features_right, 0.01, 20, NULL, 3, 0, 0.04);
    
    //cout << "Original features: " << num_last_features_left << " " << num_last_features_right << endl;
    
    //Remove ones that are too close to the edges
    for (int i = 0; i < num_last_features_left; i++)
    {
        while (last_image_features_left[i].x <= WINDOW_HALF || last_image_features_left[i].x >= PIC_WIDTH-WINDOW_HALF || last_image_features_left[i].y <= WINDOW_HALF || last_image_features_left[i].y >= PIC_HEIGHT-WINDOW_HALF )
        {
            last_image_features_left[i] =  last_image_features_left[num_last_features_left-1];
            num_last_features_left--;
            if (num_last_features_left <= 0)
                break;
        }
    }
    for (int i = 0; i < num_last_features_right; i++)
    {
        while (last_image_features_right[i].x <= WINDOW_HALF || last_image_features_right[i].x >= PIC_WIDTH-WINDOW_HALF || last_image_features_right[i].y <= WINDOW_HALF || last_image_features_right[i].y >= PIC_HEIGHT-WINDOW_HALF )
        {
            last_image_features_right[i] =  last_image_features_right[num_last_features_right-1];
            num_last_features_right--;
            if (num_last_features_right <= 0)
                break;
        }
    }
    /*  cout << "Edge removed Pts: " << endl;
    for (int i = 0; i < num_last_features_left; i++)
    {
        cout << i << ": " << last_image_features_left[i].x << " " << last_image_features_left[i].y << endl;
    }
    cout << endl;
    for (int i = 0; i < num_last_features_right; i++)
    {
        cout << i << ": " << last_image_features_right[i].x << " " << last_image_features_right[i].y << endl;
    }
    cout << endl;*/
    //cout << "Edge features removed: " << num_last_features_left << " " << num_last_features_right << endl;
    
    if (num_last_features_left == 0 || num_last_features_right == 0)
        num_features = 0;
    else
    {
        //Find stereo matches (compute SSDs and construct score matrix)
        //cout << num_last_features_left << " " << num_last_features_right << endl;
        CvMat * score = cvCreateMat(num_last_features_left,num_last_features_right,CV_32F);
        CvMat * bestmatch = cvCreateMat(num_last_features_left,num_last_features_right,CV_8U);
        int left_bestmatch[num_last_features_left];
        int right_bestmatch[num_last_features_right];
        for (int i = 0; i < num_last_features_left; i++)
        {
            for (int j = 0; j < num_last_features_right; j++)
            {
                CvPoint left_feat = cvPoint(last_image_features_left[i].x,last_image_features_left[i].y);
                CvPoint right_feat = cvPoint(last_image_features_right[j].x,last_image_features_right[j].y);

                if (abs(right_feat.y-left_feat.y) > EPI_THRESH)
                    cvmSet(score,i,j,100000.0);
                else
                {
                    cvSetImageROI(last_image[LEFT],cvRect(left_feat.x-WINDOW_HALF,left_feat.y-WINDOW_HALF,WINDOW_SIZE,WINDOW_SIZE));
                    cvSetImageROI(last_image[RIGHT],cvRect(right_feat.x-WINDOW_HALF,right_feat.y-WINDOW_HALF,WINDOW_SIZE,WINDOW_SIZE));
                    //cout << left_feat.x << " " << left_feat.y << endl;
                    //cout << right_feat.x << " " << right_feat.y << endl;

                    cvmSet(score,i,j,cvNorm(last_image[LEFT],last_image[RIGHT]));

                    cvResetImageROI(last_image[LEFT]);
                    cvResetImageROI(last_image[RIGHT]);
                }

            }
        }

        //Find left_bestmatches
        for (int i = 0; i < num_last_features_left; i++)
        {
            left_bestmatch[i] = -1;
            double current_best_cost = SSD_THRESH;
            for (int j = 0; j < num_last_features_right; j++)
            {
                CvPoint left_feat = cvPoint(last_image_features_left[i].x,last_image_features_left[i].y);
                CvPoint right_feat = cvPoint(last_image_features_right[j].x,last_image_features_right[j].y);

                if (abs(right_feat.y-left_feat.y) <= EPI_THRESH && abs(left_feat.x-right_feat.x) <= DISP_THRESH)
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
        for (int j = 0; j < num_last_features_right; j++)
        {
            right_bestmatch[j] = -1;
            double current_best_cost = SSD_THRESH;
            for (int i = 0; i < num_last_features_left; i++)
            {
                CvPoint left_feat = cvPoint(last_image_features_left[i].x,last_image_features_left[i].y);
                CvPoint right_feat = cvPoint(last_image_features_right[j].x,last_image_features_right[j].y);

                if (abs(right_feat.y-left_feat.y) <= EPI_THRESH && abs(left_feat.x-right_feat.x) <= DISP_THRESH)
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

        for (int i = 0; i < num_last_features_left; i++)
        {
            if (left_bestmatch[i] != -1 && right_bestmatch[left_bestmatch[i]] == i && (last_image_features_left[i].x - last_image_features_right[left_bestmatch[i]].x) > -2*POINT_OFFSET)
            {
                matched_features_left[k] =  last_image_features_left[i];
                matched_features_right[k] =  last_image_features_right[left_bestmatch[i]];
                k++;
            }
        }

        int num_matched_features = k;



        /*ut << "Matched Last Pts: " << endl;
        for (int i = 0; i < num_matched_features; i++)
        {
            cout << i << ": " << matched_features_left[i].x << " " << matched_features_left[i].y << " " << matched_features_right[i].x << " " << matched_features_right[i].y << endl;
        }
        cout << endl;*/



            CvSize frame_size = cvSize(PIC_WIDTH,PIC_HEIGHT);

            optical_flow_window = cvSize(25,25);
            optical_flow_termination_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, .001 );
            pyramid1 = cvCreateImage(cvSize(PIC_WIDTH+8,PIC_HEIGHT/3), IPL_DEPTH_8U, 1);
            pyramid2 = cvCreateImage(cvSize(PIC_WIDTH+8,PIC_HEIGHT/3), IPL_DEPTH_8U, 1);

        CvPoint2D32f current_image_features_left[MAX_VODOM_FEATURES];
        CvPoint2D32f current_image_features_right[MAX_VODOM_FEATURES];

        //Track to current_image left and right (track)
        cvCalcOpticalFlowPyrLK(last_image[LEFT], current_image[LEFT], pyramid1, pyramid2, &(matched_features_left[0]),&(current_image_features_left[0]), num_matched_features, optical_flow_window, 3,optical_flow_found_feature, optical_flow_feature_error,optical_flow_termination_criteria, 0 );


        //cout << "Successful tracking (left)?:\n";
        //for (int i = 0; i < num_matched_features; i++)
        //    cout << i << ": " << (double)(optical_flow_feature_error[i]) << endl;


            optical_flow_window = cvSize(25,25);
            optical_flow_termination_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, .001 );
            pyramid1 = cvCreateImage(cvSize(PIC_WIDTH+8,PIC_HEIGHT/3), IPL_DEPTH_8U, 1);
            pyramid2 = cvCreateImage(cvSize(PIC_WIDTH+8,PIC_HEIGHT/3), IPL_DEPTH_8U, 1);
        cvCalcOpticalFlowPyrLK(last_image[RIGHT], current_image[RIGHT], pyramid1, pyramid2, &(matched_features_right[0]),&(current_image_features_right[0]), num_matched_features, optical_flow_window, 3,optical_flow_found_feature, optical_flow_feature_error,optical_flow_termination_criteria, 0 );

        //cout << "Successful tracking (right)?:\n";
        //for (int i = 0; i < num_matched_features; i++)
        //    cout << i << ": " << (double)(optical_flow_feature_error[i]) << endl;


        //Copy to output vectors
        for (int i = 0; i < num_matched_features; i++)
        {
            last_stereo_pts[4*i] = matched_features_left[i].x;//+POINT_OFFSET;
            last_stereo_pts[4*i+1] = matched_features_left[i].y;
            last_stereo_pts[4*i+2] = matched_features_right[i].x;//-POINT_OFFSET;
            last_stereo_pts[4*i+3] = matched_features_right[i].y;
            current_stereo_pts[4*i] = current_image_features_left[i].x;//+POINT_OFFSET;
            current_stereo_pts[4*i+1] = current_image_features_left[i].y;
            current_stereo_pts[4*i+2] = current_image_features_right[i].x;//-POINT_OFFSET;
            current_stereo_pts[4*i+3] = current_image_features_right[i].y;
        }
        *num_features = num_matched_features;    

        //cout << "Num stereo matched features: " << num_matched_features << endl;
    }
}


void VisualOdomClass::PoseFromRANSAC(double * last_stereo_pts, double * current_stereo_pts, int num_pts, double * best_params)
{
    double params[6] = {0.0};
    //for (int i = 0; i < 6; i++)
    //    params[i] = best_params[i];
    
    double best_model_cost = HUGE_NUMBER;
    
    //Choose k sets of 3 points (where k is number of RANSAC iterations)
    //For each 3-set, find the best pose, find reprojection error for all other points, determine outliers, then fit over inlier set.
    int rand_pts[NUM_RANSAC_ITERATIONS][3] = {0};
    rand_pts[0][0] = 0;
    rand_pts[0][1] = 6;
    rand_pts[0][2] = 8;
    //cout << "Num inlier features:";
    
    for (int k = 0; k < NUM_RANSAC_ITERATIONS; k++)
    {
    double last_ransac_stereo_pts[12];
    double current_ransac_stereo_pts[12];
    double last_inlier_stereo_pts[num_pts*4];
    double current_inlier_stereo_pts[num_pts*4];
        //Randomly choose 3 numbers within number of total points (rand sampling without replacement)
        RandomSampleWithoutReplacement(num_pts, 3, rand_pts[k]);
                
        //cout << "Selected random points: " << rand_pts[k][0] << " " << rand_pts[k][1] << " " << rand_pts[k][2] << endl; 
        //Set up meas vectors, and find the optimal pose according to those 3 points
        for (int i = 0; i < 3; i++)
        {
            for (int n = 0; n < 4; n++)
            {
                last_ransac_stereo_pts[4*i+n] = last_stereo_pts[4*rand_pts[k][i]+n];
                current_ransac_stereo_pts[4*i+n] = current_stereo_pts[4*rand_pts[k][i]+n];
            }            
        }
        double model_cost = PoseFromNPts(&(last_ransac_stereo_pts[0]), &(current_ransac_stereo_pts[0]), &(params[0]),3);
        
        //Use this model to get reprojection error from all other points, and find the outliers
        double reproj_error[num_pts];
        for (int i = 0; i < num_pts; i++)
            reproj_error[i] = GetReprojectionError(&(last_stereo_pts[4*i]), &(current_stereo_pts[4*i]), &(params[0]));
        
        
        //Refit the model on the labelled inlier set
        //cout << "Inlier points: ";
        int num_inlier_pts = 0;
        for (int i = 0; i < num_pts; i++)
        {
            if (reproj_error[i] < REPROJ_OUTLIER_THRESH)
            {
                //cout << i << " ";
                for (int n = 0; n < 4; n++)
                {
                    last_inlier_stereo_pts[4*num_inlier_pts+n] = last_stereo_pts[4*i+n];
                    current_inlier_stereo_pts[4*num_inlier_pts+n] = current_stereo_pts[4*i+n];
                }
                num_inlier_pts++;
            }
        }
        
        /*cout << "Last inlier pts: " << endl;
        for (int i = 0; i < num_inlier_pts*4; i++)
            cout << last_inlier_stereo_pts[i] << " ";
        cout << endl;

        cout << "Current inlier pts: " << endl;
        for (int i = 0; i < num_inlier_pts*4; i++)
            cout << current_inlier_stereo_pts[i] << " ";
        cout << endl;
        cout << endl;*/
        
        //cout << " " << num_inlier_pts;
        if (num_inlier_pts >= MIN_INLIER_PTS)
        {
            PoseFromNPts(&(last_inlier_stereo_pts[0]), &(current_inlier_stereo_pts[0]), &(params[0]),num_inlier_pts);
            model_cost = 0.0;
            for (int i = 0; i < num_inlier_pts; i++)
            model_cost += GetReprojectionError(&(last_stereo_pts[4*i]), &(current_stereo_pts[4*i]), &(params[0]));///num_inlier_pts;
        }
        else
            model_cost = HUGER_NUMBER;
        
        if (model_cost < best_model_cost)
        {
            //cout << " (CUMULATIVE BEST)";
            best_model_cost = model_cost;
            for (int i = 0; i < 6; i++)
            {
                best_params[i] = params[i];
                //cout << " " << params[i];
            }
                
        }
        
        //cout << endl;
        
        
    }
    //cout << endl;
    //cout << "Model Reprojection Cost: " << best_model_cost <<endl;
}

double VisualOdomClass::PoseFromNPts(double * last_stereo_pts, double * current_stereo_pts, double * params, int num_pts)
 {
    double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
    int ret;
    
    /* optimization control parameters; passing to levmar NULL instead of opts reverts to defaults */
    opts[0]=1E-3; opts[1]=1E-20; opts[2]=1E-20; opts[3]=1E-20;
    opts[4]=LM_DIFF_DELTA*1E1; // relevant only if the finite difference Jacobian version is used
     ret=dlevmar_dif(vodommodelfunc, params, current_stereo_pts, 6, num_pts*4, 1000, NULL, info, NULL, NULL,  (void *)(last_stereo_pts) );
    //printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
    
        /*printf("Best fit parameters:\t");
        for (int i = 0; i < 6; i++)
            printf("%.2f ", params[i]);
        cout << endl;*/
    
    return info[1];     //Reprojection error
         }

 
void vodommodelfunc(double *params, double *current_stereo_pts, int m, int n, void *datax)
{
    double * last_stereo_pts = (double *)(datax);    
    
    CvMat * R = cvCreateMat(3,3,CV_64FC1);
    generate_Rbe(params[3],params[4],params[5],R);
    CvMat * body_pt = cvCreateMat(3,1,CV_64FC1);
    
    for (int i = 0; i < n/4; i++)
    {
        stereo_image2body(&(last_stereo_pts[4*i]),body_pt);
        body_pt->data.db[0] -= params[0];
        body_pt->data.db[1] -= params[1];
        body_pt->data.db[2] -= params[2];
        cvMatMul(R,body_pt,body_pt);
        stereo_body2image(body_pt,&(current_stereo_pts[4*i]));
    }
    cvReleaseMat(&R);
    cvReleaseMat(&body_pt);
    
    
    /*cout << "Params: " << endl;
    for (int i = 0; i < 6; i++)
        cout << params[i] << " ";
    cout << endl;
    
    cout << "Last pts: " << endl;
    for (int i = 0; i < 12; i++)
        cout << last_stereo_pts[i] << " ";
    cout << endl;
    
    cout << "Current pts: " << endl;
    for (int i = 0; i < 12; i++)
        cout << current_stereo_pts[i] << " ";
    cout << endl;
    cout << endl;*/
    
}



void stereo_image2body(double * stereo_pt, CvMat * body_pt)
{

    body_pt->data.db[0] = FX*BASELINE/(stereo_pt[0]-stereo_pt[2]);
    body_pt->data.db[1] = 0.5*(body_pt->data.db[0]/FX*(stereo_pt[0]-CX)-0.5*BASELINE)+0.5*(body_pt->data.db[0]/FX*(stereo_pt[2]-CX)+0.5*BASELINE);
    body_pt->data.db[2] = 0.5*body_pt->data.db[0]/FY*(stereo_pt[1]-CY)+0.5*body_pt->data.db[0]/FY*(stereo_pt[3]-CY);


}


void stereo_body2image(CvMat * body_pt, double * stereo_pt)
{
        stereo_pt[0] = FX*(body_pt->data.db[1]+0.5*BASELINE)/body_pt->data.db[0]+CX;
	stereo_pt[1] = FY*(body_pt->data.db[2]/body_pt->data.db[0])+CY;
	stereo_pt[2] = FX*(body_pt->data.db[1]-0.5*BASELINE)/body_pt->data.db[0]+CX;
	stereo_pt[3] = FY*(body_pt->data.db[2]/body_pt->data.db[0])+CY;
}

//Choose n integers in range [0, m-1]
void VisualOdomClass::RandomSampleWithoutReplacement(int m, int n, int * samples)
{
    int num_chosen = 0;
    
    for (int i = 0; i < m; i++)
    {
        double prob = (n-num_chosen)/(m-i);
        if (rand() % (m-i) <= (n-num_chosen))
        {
            samples[num_chosen++] = i;
        }
        if (num_chosen == n)
            break;
    }
}

double VisualOdomClass::GetReprojectionError(double * last_stereo_pt, double * current_stereo_pt, double * params)
{
    CvMat * R = cvCreateMat(3,3,CV_64FC1);
    generate_Rbe(params[3],params[4],params[5],R);
    CvMat * body_pt = cvCreateMat(3,1,CV_64FC1);
    
    double projected_stereo_pt[4];
    
    stereo_image2body(last_stereo_pt,body_pt);
    body_pt->data.db[0] -= params[0];
    body_pt->data.db[1] -= params[1];
    body_pt->data.db[2] -= params[2];
    cvMatMul(R,body_pt,body_pt);
    stereo_body2image(body_pt,projected_stereo_pt);
    
    cvReleaseMat(&R);
    cvReleaseMat(&body_pt);
    
    double error = 0.0;
    
    for (int i = 0; i < 4; i++)
    {
        error += pow( current_stereo_pt[i]-projected_stereo_pt[i] , 2.0);
    }
    /*cout << "Last Point: ";
    for (int i = 0; i < 4; i++)
        cout << last_stereo_pt[i] << " ";
    cout << "Current Point: ";
    for (int i = 0; i < 4; i++)
        cout << current_stereo_pt[i] << " ";
    cout << "Projected Point: ";
    for (int i = 0; i < 4; i++)
        cout << projected_stereo_pt[i] << " ";
    
           cout << "Reprojection error: " << error << endl;*/
    return error;
}