#include "displayClass.h"
#include "curveFittingClass.h"
#include "common.h"
#include "kalmanClass.h"

using namespace std;


DisplayClass::DisplayClass()
{
        cvNamedWindow("Landmark Map");
        cvMoveWindow("Landmark Map",0,0);
        cvNamedWindow("Left");
        cvMoveWindow("Left",0,0);

        image_color_left = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U, 1);
        cvNamedWindow("Right");
        cvMoveWindow("Right",0,0);
        
        image_color_right = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U, 1);

        cumulative_path = cvCreateImage(cvSize(MAP_IMAGE_WIDTH,MAP_IMAGE_HEIGHT),IPL_DEPTH_8U, 3);
        output_image = cvCreateImage(cvSize(2.0*MAP_IMAGE_WIDTH,PIC_HEIGHT+MAP_IMAGE_HEIGHT),IPL_DEPTH_8U, 3);

        //video_out = cvCreateVideoWriter( "Output.avi", CV_FOURCC('P','I','M','1'), 30.0, cvSize(MAP_IMAGE_WIDTH,PIC_HEIGHT+MAP_IMAGE_HEIGHT), 1 );



        landmark_map = cvCreateImage(cvSize(MAP_IMAGE_WIDTH,MAP_IMAGE_HEIGHT),IPL_DEPTH_8U,3);
        cvSet(landmark_map, cvScalar(255,255,255));
        cvSet(cumulative_path, cvScalar(255,255,255));


	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);

}

DisplayClass::~DisplayClass()
{


        cvReleaseImage(&landmark_map);
}


float DisplayClass::getTime()
{
    return elapsedTime;
}

void DisplayClass::resetTime()
{
    elapsedTime = 0.0;
}



//Convert from 3D point to an image point for landmark map
void DisplayClass::convert3Dtogroundmap(CvPoint3D32f * world_point, CvPoint * map_point)
{
    float x = world_point->x;
    float y = world_point->y;

    map_point->x = MAP_IMAGE_WIDTH*( 0.5 + y/MAP_PHYSICAL_WIDTH );
    map_point->y = MAP_IMAGE_HEIGHT*( 0.9 - x/MAP_PHYSICAL_HEIGHT );
}



//Generate landmark map at each timestep
void DisplayClass::generate_map(CvMat * state, std::vector<double> * state_limits, CvMat * z, int num_curves, int num_pts)
{
	gettimeofday(&start, NULL);
        
                CvPoint map_point;


        CvPoint3D32f pos;

        //Plot robot pose
        pos.x = state->data.db[0];
        pos.y = state->data.db[1];
        pos.z = state->data.db[5];

        CvPoint fwd_point;
        CvPoint left_point;
        CvPoint right_point;
        convert3Dtogroundmap(&pos, &map_point);
        cvCircle(cumulative_path, map_point,1,CV_RGB(255,0,0));
        cvCopy(cumulative_path,landmark_map,NULL);
        fwd_point.x = map_point.x+30.0*sin(pos.z);
        fwd_point.y = map_point.y-30.0*cos(pos.z);
        left_point.x = map_point.x-10.0*cos(pos.z);
        left_point.y = map_point.y-10.0*sin(pos.z);
        right_point.x = map_point.x+10.0*cos(pos.z);
        right_point.y = map_point.y+10.0*sin(pos.z);

        cvLine(landmark_map,fwd_point,right_point, CV_RGB(255,0,0),2,1,0);
        cvLine(landmark_map,fwd_point,left_point, CV_RGB(255,0,0),2,1,0);
        cvLine(landmark_map,left_point,right_point, CV_RGB(255,0,0),2,1,0);

                   pos.x = 0.0;
                   pos.y = 0.0;
                   pos.z = 0.0;

        
        //Generate state map curves
        for (int k = 0; k < num_curves; k++)
        {

            for (double t = 0.0; t <= state_limits->at(k); t += 0.001)
            {

                    for (int i = 0; i <= 3; i++)
                    {
                        pos.x += binom[3][i]*pow(1-t,3-i)*pow(t,i)*state->data.db[8*k+i+ROBOT_STATE_SIZE]; //+3 because of the 3D planar pose state variables
                        pos.y += binom[3][i]*pow(1-t,3-i)*pow(t,i)*state->data.db[8*k+i+ROBOT_STATE_SIZE+4];
                    }


                   convert3Dtogroundmap(&pos, &map_point);

                   cvCircle(landmark_map,map_point,1,CV_RGB(0,0,0));
                   pos.x = 0.0;
                   pos.y = 0.0;
            }
        }

        //Generate measured curves
        for (int k = 0; k < 4; k++)
        {

            for (double t = 0.0; t <= 1.0; t += 0.001)
            {

                    for (int i = 0; i <= 3; i++)
                    {
                        pos.x += binom[3][i]*pow(1-t,3-i)*pow(t,i)*z->data.db[8*k+i];
                        pos.y += binom[3][i]*pow(1-t,3-i)*pow(t,i)*z->data.db[8*k+i+4];
                    }


                   convert3Dtogroundmap(&pos, &map_point);

                   //if (k < 2)
                    //cvCircle(landmark_map,map_point,1,CV_RGB(255,0,0));
                   //else
                    //cvCircle(landmark_map,map_point,1,CV_RGB(0,0,255));
                   pos.x = 0.0;
                   pos.y = 0.0;
            }
        }

    cvShowImage("Landmark Map",landmark_map);

    cvSaveImage ("map.jpg", landmark_map);

    cvWaitKey(10);



#ifdef OUTPUT_VIDEO
    cvSetImageROI(output_image, cvRect(0,PIC_HEIGHT,MAP_IMAGE_WIDTH,MAP_IMAGE_HEIGHT));
    cvCopy(landmark_map,output_image, NULL);
    cvSetImageROI(output_image, cvRect(MAP_IMAGE_WIDTH,PIC_HEIGHT,2.0*MAP_IMAGE_WIDTH,MAP_IMAGE_HEIGHT));
   // cvCopy(height_map,output_image, NULL);
#endif


        gettimeofday(&stop, NULL);
	elapsedTime = (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
}


void DisplayClass::display_images(IplImage* image_left, IplImage* image_right, CvMat ** featuresLeftImage, CvMat ** featuresRightImage, double * p_left, double * p_right, IplImage * image_out_left, IplImage * image_out_right)
{
	gettimeofday(&start, NULL);


	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,8);

#ifndef USE_SIMULATION

    cvCopy(image_left, image_color_left,NULL);
    cvCopy(image_right, image_color_right,NULL);


    // Plot features on images

    char num_string[10];

    
    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
           double xval = p_left[0]*pow(t,3.0)+p_left[1]*pow(t,2.0)+p_left[2]*t+p_left[3]-POINT_OFFSET;
           double yval = p_left[4]*pow(t,3.0)+p_left[5]*pow(t,2.0)+p_left[6]*t+p_left[7];

           //cvCircle(image_color_left,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    		//cvCircle(image_out_left,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    }
    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
           double xval = p_right[0]*pow(t,3.0)+p_right[1]*pow(t,2.0)+p_right[2]*t+p_right[3]+POINT_OFFSET;
           double yval = p_right[4]*pow(t,3.0)+p_right[5]*pow(t,2.0)+p_right[6]*t+p_right[7];

           //cvCircle(image_color_right,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    		//cvCircle(image_out_right,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    }


    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
           double xval = p_left[8]*pow(t,3.0)+p_left[9]*pow(t,2.0)+p_left[10]*t+p_left[11]-POINT_OFFSET;
           double yval = p_left[12]*pow(t,3.0)+p_left[13]*pow(t,2.0)+p_left[14]*t+p_left[15];

           //cvCircle(image_color_left,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    		//cvCircle(image_out_left,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    }
    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
           double xval = p_right[8]*pow(t,3.0)+p_right[9]*pow(t,2.0)+p_right[10]*t+p_right[11]+POINT_OFFSET;
           double yval = p_right[12]*pow(t,3.0)+p_right[13]*pow(t,2.0)+p_right[14]*t+p_right[15];

           //cvCircle(image_color_right,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    		//cvCircle(image_out_right,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    }
/*
    //Replot the answer we get back to images
    double p[] = {-0.2878826, 1.136295, 1.258331, 2.226922, 0.3540906, -0.3593112, 0.07408991, -1.144323};
    double p1_left[8], p1_right[8];
    double euler[] = {0.0, -0.009630287, 0.0};
    double translation[] = {0.0,0.0,-1.008484};

    poly_earth2image(p,p1_left,p1_right,euler,translation);

    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
           double xval = p1_left[0]*pow(t,3.0)+p1_left[1]*pow(t,2.0)+p1_left[2]*t+p1_left[3];
           double yval = p1_left[4]*pow(t,3.0)+p1_left[5]*pow(t,2.0)+p1_left[6]*t+p1_left[7];

           //cvCircle(image_color_left,cvPoint(xval,yval),1,CV_RGB(155,155,155));
    }
    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
           double xval = p1_right[0]*pow(t,3.0)+p1_right[1]*pow(t,2.0)+p1_right[2]*t+p1_right[3];
           double yval = p1_right[4]*pow(t,3.0)+p1_right[5]*pow(t,2.0)+p1_right[6]*t+p1_right[7];

           //cvCircle(image_color_right,cvPoint(xval,yval),1,CV_RGB(155,155,155));
    }*/


    //cvSaveImage ("edge0.jpg", image_color_left);
    //cvCvtColor(image_color_left,img_out_left,CV_GRAY2RGB);
    //cvCvtColor(image_color_right,img_out_right,CV_GRAY2RGB);

/*
    for( int i = 0; i < num_feat; i++)
    {
        int i_right = features_left->at(i).stereo_match;

        if (i_right != -1) //Only plot the features if there's a stereo match
        {
            sprintf(num_string,"%d",i);
            CvPoint left = features_left->at(i).image_pt;
            cvCircle(image_color_left,left,1,CV_RGB(255,255,255));
            left.x += 10;
            cvPutText(image_color_left,num_string,left,&font,CV_RGB(0,0,0));

            CvPoint right = features_right->at(i_right).image_pt;
            cvCircle(image_color_right,right,1,CV_RGB(255,255,255));
            right.x += 10;
            cvPutText(image_color_right,num_string,right,&font,CV_RGB(0,0,0));
        }
    }
*/
// Show output
    //cvShowImage("Left",image_color_left);

    //cvShowImage("Right",image_color_right);


#endif

#ifdef OUTPUT_VIDEO
    cvSetImageROI(output_image, cvRect(0,0,PIC_WIDTH,PIC_HEIGHT));
    //cvCvtColor(image_color_left,output_image,CV_GRAY2RGB);
    cvSetImageROI(output_image, cvRect(PIC_WIDTH,0,PIC_WIDTH,PIC_HEIGHT));
    //cvCvtColor(image_color_right,output_image,CV_GRAY2RGB);
    cvResetImageROI(output_image);
    cvWriteFrame(video_out,output_image);
#endif
    

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);

}










void DisplayClass::display_images(IplImage* image_left, IplImage* image_right, vector<Keypoint> *features_left, vector<Keypoint> *features_right)
{
	gettimeofday(&start, NULL);


	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,8);

#ifndef USE_SIMULATION

    cvCopy(image_left, image_color_left,NULL);
    cvCopy(image_right, image_color_right,NULL);


    // Plot features on images

    char num_string[10];

    int num_feat = features_right->size();

    //cout << "RIGHT FEATURES\n";
    for( int i = 0; i < num_feat; i++)
    {
            //sprintf(num_string,"%d",i);
            CvPoint right = features_right->at(i).image_pt;
            cvCircle(image_color_right,right,1,CV_RGB(0,0,0));

            if (features_right->at(i).stereo_match == -1)
            {
                //right.x += 10;
                //cvPutText(image_color_right,num_string,right,&font,CV_RGB(0,0,0));
            }
            //printf("Feature num: %d\t Match: %d\n",i, features_right->at(i).stereo_match);
    }

    num_feat = features_left->size();

    //cout << "LEFT FEATURES\n";
    for( int i = 0; i < num_feat; i++)
    {
            //sprintf(num_string,"%d",i);
            CvPoint left = features_left->at(i).image_pt;
            cvCircle(image_color_left,left,1,CV_RGB(0,0,0));

            if (features_left->at(i).stereo_match == -1)
            {
                //left.x += 10;
                //cvPutText(image_color_left,num_string,left,&font,CV_RGB(0,0,0));
            }
            //printf("Feature num: %d\t Match: %d\n",i, features_left->at(i).stereo_match);
    }


    //cvSaveImage ("edge0.jpg", image_color_left);

/*
    for( int i = 0; i < num_feat; i++)
    {
        int i_right = features_left->at(i).stereo_match;

        if (i_right != -1) //Only plot the features if there's a stereo match
        {
            sprintf(num_string,"%d",i);
            CvPoint left = features_left->at(i).image_pt;
            cvCircle(image_color_left,left,1,CV_RGB(255,255,255));
            left.x += 10;
            cvPutText(image_color_left,num_string,left,&font,CV_RGB(0,0,0));

            CvPoint right = features_right->at(i_right).image_pt;
            cvCircle(image_color_right,right,1,CV_RGB(255,255,255));
            right.x += 10;
            cvPutText(image_color_right,num_string,right,&font,CV_RGB(0,0,0));
        }
    }
*/
// Show output
    //cvShowImage("Left",image_color_left);

    //cvShowImage("Right",image_color_right);


#endif

#ifdef OUTPUT_VIDEO
    cvSetImageROI(output_image, cvRect(0,0,PIC_WIDTH,PIC_HEIGHT));
    cvCvtColor(image_color_left,output_image,CV_GRAY2RGB);
    cvSetImageROI(output_image, cvRect(PIC_WIDTH,0,PIC_WIDTH,PIC_HEIGHT));
    cvCvtColor(image_color_right,output_image,CV_GRAY2RGB);
    cvResetImageROI(output_image);
#endif
    //cvWriteFrame(video_out,output_image);

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);

}

void DisplayClass::copy_images(IplImage ** image)
{

    cvCopy(image[0], image_color_left,NULL);
    cvCopy(image[1], image_color_right,NULL);
}




void DisplayClass::display_images(double *p_left, double * p_right)
{
	gettimeofday(&start, NULL);


	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,8);

#ifndef USE_SIMULATION

    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
           double xval = p_left[0]*pow(t,3.0)+p_left[1]*pow(t,2.0)+p_left[2]*t+p_left[3];
           double yval = p_left[4]*pow(t,3.0)+p_left[5]*pow(t,2.0)+p_left[6]*t+p_left[7];

           cvCircle(image_color_left,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    }
    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
           double xval = p_right[0]*pow(t,3.0)+p_right[1]*pow(t,2.0)+p_right[2]*t+p_right[3];
           double yval = p_right[4]*pow(t,3.0)+p_right[5]*pow(t,2.0)+p_right[6]*t+p_right[7];

           cvCircle(image_color_right,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    }


    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
           double xval = p_left[8]*pow(t,3.0)+p_left[9]*pow(t,2.0)+p_left[10]*t+p_left[11];
           double yval = p_left[12]*pow(t,3.0)+p_left[13]*pow(t,2.0)+p_left[14]*t+p_left[15];

           cvCircle(image_color_left,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    }
    for (double t = 0.0; t <= 1.0; t += 0.01)
    {
           double xval = p_right[8]*pow(t,3.0)+p_right[9]*pow(t,2.0)+p_right[10]*t+p_right[11];
           double yval = p_right[12]*pow(t,3.0)+p_right[13]*pow(t,2.0)+p_right[14]*t+p_right[15];

           cvCircle(image_color_right,cvPoint(xval,yval),1,CV_RGB(255,255,255));
    }


// Show output
    cvShowImage("Left",image_color_left);

    cvShowImage("Right",image_color_right);


#endif


}