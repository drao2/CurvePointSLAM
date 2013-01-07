#include "dataAssocClass.h"
#include "curveFittingClass.h"

using namespace std;


void plotGraph(double * x, double * y, int num_pts, int width, int height)
{
    cvNamedWindow("Graph");
    cvMoveWindow("Graph",0,0);

    IplImage * graph_image = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);

    double x_max = 0.0; double y_max = 0.0;
    for (int i = 0; i < num_pts; i++)
    {
        if (y[i] > y_max)            y_max = y[i];
        if (x[i] > x_max)            x_max = x[i];
    }
    y_max = 2.0;//y_max >= x_max ? y_max : x_max;

    for (int i = 0; i < num_pts; i++)
    {
        CvPoint plot_point = cvPoint(i*width/num_pts, height-0.75*x[i]*height/y_max);
        cvCircle(graph_image, plot_point,1,CV_RGB(0,255,0));
        plot_point = cvPoint(i*width/num_pts, height-0.75*y[i]*height/y_max);
        cvCircle(graph_image, plot_point,1,CV_RGB(255,0,0));
    }

    cvShowImage("Graph",graph_image);
    cvWaitKey(10);

}

void plotGraph(double * x, double * y, double * z, int num_pts, int width, int height)
{
    cvNamedWindow("Graph");
    cvMoveWindow("Graph",0,0);

    IplImage * graph_image = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);

    double x_max = 0.0; double y_max = 0.0; double z_max = 0.0;
    for (int i = 0; i < num_pts; i++)
    {
        if (y[i] > y_max)            y_max = y[i];
        if (x[i] > x_max)            x_max = x[i];
        if (z[i] > z_max)            z_max = z[i];
    }
    y_max = 2.0;//y_max >= x_max ? y_max : x_max;

    for (int i = 0; i < num_pts; i++)
    {
        CvPoint plot_point = cvPoint(i*width/num_pts, height-0.75*x[i]*height/y_max);
        cvCircle(graph_image, plot_point,1,CV_RGB(255,0,0));
        plot_point = cvPoint(i*width/num_pts, height-0.75*y[i]*height/y_max);
        cvCircle(graph_image, plot_point,1,CV_RGB(0,255,0));
        plot_point = cvPoint(i*width/num_pts, height-0.75*z[i]*height/y_max);
        cvCircle(graph_image, plot_point,1,CV_RGB(0,0,255));
    }

    cvShowImage("Graph",graph_image);
    cvWaitKey(10);

}

DataAssocClass::DataAssocClass()
{
	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);



    frame_size = cvSize(PIC_WIDTH,PIC_HEIGHT);

    optical_flow_window = cvSize(PIC_WIDTH/5-1,PIC_WIDTH/5-1);
    optical_flow_termination_criteria = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, .0001 );


    eig_image = NULL;
    temp_image = NULL;
    pyramid1 = NULL;
    pyramid2 = NULL;
    
    allocateOnDemand( &pyramid1, frame_size, IPL_DEPTH_8U, 1 );
    allocateOnDemand( &pyramid2, frame_size, IPL_DEPTH_8U, 1 );
    allocateOnDemand( &eig_image, frame_size, IPL_DEPTH_32F, 1 );
    allocateOnDemand( &temp_image, frame_size, IPL_DEPTH_32F, 1 );


    tracking_started = false;
    map_endpt_tracked.y = 1000.0;
    
    last_m = 1.0;
    last_b = 0.0;
    current_m = 1.0;
    current_b = 0.0;
    
    
    A = cvCreateMat(NUM_VODOM_PTS,2,CV_64FC1);
    At = cvCreateMat(2,NUM_VODOM_PTS,CV_64FC1);
    AtA = cvCreateMat(2,2,CV_64FC1);
    AtAinv = cvCreateMat(2,2,CV_64FC1);
    b = cvCreateMat(NUM_VODOM_PTS,1,CV_64FC1);
    xparams = cvCreateMat(2,1,CV_64FC1);

}

DataAssocClass::~DataAssocClass()
{

}

void DataAssocClass::singleFrameTrack(IplImage ** last_image, IplImage ** image, CvRect roi)
{
    IplImage * prev_image = cvCloneImage(last_image[0]);
    IplImage * current_image = cvCloneImage(image[0]);
    cvSetImageROI(last_image[0],roi);
    cvSetImageROI(image[0],roi);
    if (tracking_started)
    {
        for (int i = 0; i < NUM_VODOM_PTS; i++)
        {
                cvCircle(prev_image, cvPoint(vodom_pts_tracked[i].x,vodom_pts_tracked[i].y), 5, CV_RGB(0,0,255), 1, CV_AA, 0 );
                vodom_pts_tracked[i].x = vodom_pts_tracked[i].x -roi.x;
                vodom_pts_tracked[i].y = vodom_pts_tracked[i].y -roi.y;
                cvCalcOpticalFlowPyrLK(last_image[0], image[0], pyramid1, pyramid2, &(vodom_pts_tracked[i]),&(temp_pt), 1, optical_flow_window, 1,optical_flow_found_feature, optical_flow_feature_error,optical_flow_termination_criteria, 0 );
                vodom_pts_tracked[i].x = temp_pt.x +roi.x;
                vodom_pts_tracked[i].y = temp_pt.y +roi.y;
                cvCircle(current_image, cvPoint(vodom_pts_tracked[i].x,vodom_pts_tracked[i].y), 5, CV_RGB(0,0,255), 1, CV_AA, 0 );
        }

        
        
        
        temp_pt.x = map_endpt_tracked.x;
        temp_pt.y = map_endpt_tracked.y;
        
        cvCalcOpticalFlowPyrLK(last_image[0], image[0], pyramid1, pyramid2, &(temp_pt),&(map_endpt_tracked), 1, optical_flow_window, 1,optical_flow_found_feature, optical_flow_feature_error,optical_flow_termination_criteria, 0 );
        cvCalcOpticalFlowPyrLK(last_image[0], image[0], pyramid1, pyramid2, &(temp_pt6),&(temp_pt6a), 1, optical_flow_window, 1,optical_flow_found_feature, optical_flow_feature_error,optical_flow_termination_criteria, 0 );

    
    //cvShowImage("Left",current_image);
    //cvShowImage("Last Left",prev_image);
    //cvWaitKey(0);
    }
    cvResetImageROI(last_image[0]);
    cvResetImageROI(image[0]);
    
    cvReleaseImage(&prev_image);
    cvReleaseImage(&current_image);


    
}


double * DataAssocClass::getMatchT(CvMat * curve_features,  IplImage ** last_image, IplImage ** last_image_track, IplImage ** image, IplImage ** last_image_color, IplImage ** image_color, double * last_params, double * params, int curve_num, char * data_assoc_flag_ptr, bool reset_data_assoc)
{

	gettimeofday(&start, NULL);

    //last_image = last image on which SLAM was performed, last_image_track = last available frame

    char data_assoc_flag = *data_assoc_flag_ptr;
    tracking_started = true;

    for (int i = 0; i < 3; i++)
        last_t_split[i] = t_split[i];


    int features_rows = curve_features->rows;



   
    //KEEP TRACK OF CURVE ENDS!

    if (reset_data_assoc)
    {
        meas_endpt.x = curve_features->data.db[features_rows-2];
        meas_endpt.y = curve_features->data.db[features_rows-1];
        meas_startpt.x = curve_features->data.db[0];
        meas_startpt.y = curve_features->data.db[1];

        map_endpt.x = curve_features->data.db[features_rows-2];
        map_endpt.y = curve_features->data.db[features_rows-1];
        map_endpt_tracked.x = curve_features->data.db[features_rows-2];
        map_endpt_tracked.y = curve_features->data.db[features_rows-1];

        map_endpt_match.x = map_endpt.x;
        map_endpt_match.y = map_endpt.y;
                
        temp_pt6a = meas_endpt;
    }
    else
    {
        last_meas_endpt.x = meas_endpt.x;
        last_meas_endpt.y = meas_endpt.y;
        last_meas_startpt.x = meas_startpt.x;
        last_meas_startpt.y = meas_startpt.y;


        meas_endpt.x = curve_features->data.db[features_rows-2];
        meas_endpt.y = curve_features->data.db[features_rows-1];
        meas_startpt.x = curve_features->data.db[0];
        meas_startpt.y = curve_features->data.db[1];

        cvCalcOpticalFlowPyrLK(image[0], last_image[0], pyramid1, pyramid2, &(meas_startpt),&(meas_startpt_match), 1, optical_flow_window, 1,optical_flow_found_feature, optical_flow_feature_error,optical_flow_termination_criteria, 0 );

        map_endpt_match.x = map_endpt_tracked.x;
        map_endpt_match.y = map_endpt_tracked.y;

    }
 
   temp_pt6 = meas_endpt;




    //cvCircle(image_color[0], cvPoint(map_endpt_match.x,map_endpt_match.y), 5, CV_RGB(0,0,255), 1, CV_AA, 0 );
    //cvCircle(last_image_color[0], cvPoint(map_endpt.x,map_endpt.y), 5, CV_RGB(0,0,255), 1, CV_AA, 0 );
    cvCircle(image_color[0], cvPoint(meas_startpt.x,meas_startpt.y), 5, CV_RGB(255,0,0), 1, CV_AA, 0 );
    cvCircle(image_color[0], cvPoint(meas_endpt.x,meas_endpt.y), 5, CV_RGB(255,0,0), 1, CV_AA, 0 );
    //cvCircle(last_image_color[0], cvPoint(meas_startpt_match.x,meas_startpt_match.y), 5, CV_RGB(255,0,0), 1, CV_AA, 0 );

    //cvCircle(image_color[0], cvPoint(temp_pt2a.x,temp_pt2a.y), 5, CV_RGB(0,0,255), 1, CV_AA, 0 );
    //cvCircle(image_color[0], cvPoint(temp_pt3a.x,temp_pt3a.y), 5, CV_RGB(0,0,255), 1, CV_AA, 0 );
    //cvCircle(image_color[0], cvPoint(temp_pt4a.x,temp_pt4a.y), 5, CV_RGB(0,0,255), 1, CV_AA, 0 );
    //cvCircle(image_color[0], cvPoint(temp_pt5a.x,temp_pt5a.y), 5, CV_RGB(0,0,255), 1, CV_AA, 0 );
    //cvCircle(image_color[0], cvPoint(temp_pt6a.x+pt_diff.x,temp_pt6a.y+pt_diff.y), 5, CV_RGB(0,0,255), 1, CV_AA, 0 );

    
        //for (int i = 0; i < NUM_TRACK_PTS; i++)
        //{
            //cvCircle(image_color[0], cvPoint(temp_pt.x+5*(track_pts_a[i].x-track_pts[i].x),temp_pt.y+5*(track_pts_a[i].y-track_pts[i].y)), 5, CV_RGB(0,0,255), 1, CV_AA, 0 );
        //}
    
    
    map_endpt.x = map_endpt_tracked.x;
    map_endpt.y = map_endpt_tracked.y;


    double split_pt[] = {map_endpt_match.x,map_endpt_match.y};
    double start_pt[] = {meas_startpt_match.x,meas_startpt_match.y};
    double out[2];
    double actual_split_pt[2];



    double euler[3];
    double translation[3];
    double p[8], p_left[8], p_right[8];
    
    euler[0] = 0.0;
    euler[1] = params[16];
    euler[2] = params[17];
    translation[0] = 0.0;
    translation[1] = 0.0;
    translation[2] = params[18];

    control2coeffs(&(params[curve_num*8]),p);
    poly_earth2image(p, p_left, p_right, euler, translation);
    
    
    
    
    for (int i = 0; i < NUM_VODOM_PTS; i++)
    {
        double vodom_pt[] = {vodom_pts_tracked[i].x,vodom_pts_tracked[i].y};
        closest_bezier_pt(p_left,&(vodom_pt[0]), &(actual_split_pt[0]), &(out[0]));
        vodom_t_vals[i][1] = out[1];
        vodom_t_vals[i][0] = 1-0.1*(i+1);
        vodom_pts_tracked[i] = cvPoint2D32f(p_left[0]*pow(vodom_t_vals[i][0],3.0)+p_left[1]*pow(vodom_t_vals[i][0],2.0)+p_left[2]*vodom_t_vals[i][0]+p_left[3] , p_left[4]*pow(vodom_t_vals[i][0],3.0)+p_left[5]*pow(vodom_t_vals[i][0],2.0)+p_left[6]*vodom_t_vals[i][0]+p_left[7]);
        //cout << vodom_t_vals[i][0] << "->" << vodom_t_vals[i][1] << endl;
    }
    
    cvSetZero(A);
    cvSetZero(b);
    double tz = -1.0;
if (!reset_data_assoc)
{
    for (int i = 0; i < NUM_VODOM_PTS; i++)
    {
        if (fabs(vodom_t_vals[i][1]) > 0.001)
        {
            cvmSet(A,i,0,vodom_t_vals[i][0]);
            cvmSet(A,i,1,1.0);
        }
        cvmSet(b,i,0,vodom_t_vals[i][1]);
    }

    cvTranspose(A,At);
    cvMatMul(At,A,AtA);
    cvInvert(AtA,AtAinv,CV_LU);
    cvMatMul(At,b,xparams);
    cvMatMul(AtAinv,xparams,xparams);
    
    current_m = last_m*xparams->data.db[0];
    current_b = xparams->data.db[0]*last_b+xparams->data.db[1];
    last_m = current_m;
    last_b = current_b;
    //current_m = xparams->data.db[0];
    //current_b = xparams->data.db[1];
    //cout << "New: ti = " << -current_b/current_m << "\ttz = " << current_m+current_b << "\ttj = " << (1.0-current_b)/current_m << endl;
    
    tz = current_m+current_b;
    double new_map_endpt[] = {p_left[0]*pow(tz,3.0)+p_left[1]*pow(tz,2.0)+p_left[2]*tz+p_left[3] , p_left[4]*pow(tz,3.0)+p_left[5]*pow(tz,2.0)+p_left[6]*tz+p_left[7]};
    map_endpt_match.x = new_map_endpt[0];
    map_endpt_match.y = new_map_endpt[1];
    cvCircle(image_color[0], cvPoint(new_map_endpt[0],new_map_endpt[1]), 5, CV_RGB(0,255,0), 1, CV_AA, 0 );
    
}
else
{
    current_m = 1.0;
    current_b = 0.0;
    last_m = current_m;
    last_b = current_b;
}
    
    
    
    //x1a = p[0]*pow(tt,3.0)+p[1]*pow(tt,2.0)+p[2]*tt+p[3];
    //y1a = p[4]*pow(tt,3.0)+p[5]*pow(tt,2.0)+p[6]*tt+p[7];
    
    
    
    //if (pow( pow( temp_pt6a.x+pt_diff.x-map_endpt_tracked.x , 2.0 ) + pow( temp_pt6a.y+pt_diff.y-map_endpt_tracked.y , 2.0 ) , 0.5 )> DATA_ASSOC_THRESHOLD)
    //{
    //    t_split[0] = -1.0;
    //    t_split[1] = -1.0;
    //    t_split[2] = -1.0;
    //}
    //else if (reset_data_assoc)
    //{
    //    t_split[0] = -1.0;
    //    t_split[1] = -1.0;
    //    t_split[2] = -1.0;
    //}
    if(reset_data_assoc)
    {
        t_split[0] = -1.0;
        t_split[1] = -1.0;
        t_split[2] = -1.0;
    }
    else
    {
        if (DIST(meas_startpt,map_endpt_match) < 5)
        {
            t_split[0] = 1.0;
            t_split[1] = 0.0;
            t_split[2] = 1.0;
        }
        else if (DIST(meas_endpt,map_endpt_match) < 5)
        {
            t_split[0] = 0.0;
            t_split[1] = 1.0;
            t_split[2] = 0.0;

        }
        else
        {
            closest_bezier_pt(p_left,&(split_pt[0]), &(actual_split_pt[0]), &(out[0]));
                t_split[1] = out[1];
                t_split[1] = MAX(MIN(tz,1.0),0.0);
                map_endpt_tracked.x = actual_split_pt[0];
                map_endpt_tracked.y = actual_split_pt[1];

            if (data_assoc_flag & (ONLY_ADD_STATE|ONLY_UPDATE_ONE_STATE|ADD_FIRST_STATES))
            {
                t_split[2] = (1-t_split[0])*(1-t_split[1])/t_split[1];
            }
            else
            {
                t_split[0] = 1 - ((last_t_split[1]-t_split[0])*last_t_split[2]/(1-last_t_split[1]));

                t_split[2] = (1-t_split[0])*(1-t_split[1])/t_split[1];
            }
        }

        t_split[0] = 1-t_split[1];
        t_split[2] = 1-t_split[1];

    }
        double old_map_endpt[] = {p_left[0]*pow(t_split[1],3.0)+p_left[1]*pow(t_split[1],2.0)+p_left[2]*t_split[1]+p_left[3] , p_left[4]*pow(t_split[1],3.0)+p_left[5]*pow(t_split[1],2.0)+p_left[6]*t_split[1]+p_left[7]};
    
    cvCircle(image_color[0], cvPoint(old_map_endpt[0],old_map_endpt[1]), 5, CV_RGB(0,0,255), 1, CV_AA, 0 );
    
    //cout << "Old: ti = " <<  t_split[0] << "\ttz = " <<  t_split[1] << "\tj = " <<  t_split[2] << endl;


	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);




    return &(t_split[0]);
}

void DataAssocClass::updateMapCurves()
{
    map_endpt_match.x = meas_endpt.x;
    map_endpt_match.y = meas_endpt.y;
    map_endpt_tracked.x = meas_endpt.x;
    map_endpt_tracked.y = meas_endpt.y;
    map_endpt.x = meas_endpt.x;
    map_endpt.y = meas_endpt.y;
    //current_m = xparams->data.db[0];
    //current_b = xparams->data.db[1];
    current_m = 1.0;
    current_b = 0.0;
    last_m = 1.0;
    last_b = 0.0;
}



float DataAssocClass::getTime()
{
    return elapsedTime;
}

void DataAssocClass::resetTime()
{
    elapsedTime = 0.0;
}
