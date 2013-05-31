#include "featureDetector.h"
#include "capture.h"
//#include "cvblobs/BlobResult.h"
//#include "cvblobs/BlobProperties.h"

using namespace std;



FeatureDetector::FeatureDetector()
{
	gettimeofday(&start, NULL);
        
	size = cvSize(PIC_WIDTH,PIC_HEIGHT);
	eig_image = cvCreateImage(size,IPL_DEPTH_32F,1);
	edge = cvCreateImage(size,IPL_DEPTH_8U,1);
	edge_clr = cvCreateImage(size,IPL_DEPTH_8U,3);
        img = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);
        edge_search_region = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);
        last_edge_search_region = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);

        img_hsv = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,3);
        img_h = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);
        img_s = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);
        img_v = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);

        prev_left_x = PIC_WIDTH/2-20;
        prev_left_y = PIC_HEIGHT-1;
        prev_right_x = PIC_WIDTH/2+20;
        prev_right_y = PIC_HEIGHT-1;


        featuresLeftCurve.reserve(1000);
        featuresRightCurve.reserve(1000);

        

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
}

FeatureDetector::~FeatureDetector()
{
	gettimeofday(&start, NULL);
        cvReleaseImage(&edge);
	cvReleaseImage(&edge_clr);

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
}


void FeatureDetector::find_features(IplImage* img, IplImage* seg_img, int cam, CvRect * roi, CvPoint2D32f * map_endpts)
{
	gettimeofday(&start, NULL);

#ifdef EDGE

        cvCvtColor(img,img_hsv,CV_RGB2HSV);

        //DO EDGE FOLLOWING TO SELECT SPECIFIC EDGES ONLY
        char string[20];
        cvSetImageCOI(img_hsv,1);
        cvCopy(img_hsv,img_h,NULL);
        cvSetImageCOI(img_hsv,2);
        cvCopy(img_hsv,img_s,NULL);
        cvSetImageCOI(img_hsv,3);
        cvCopy(img_hsv,img_v,NULL);
        cvSetImageCOI(img_hsv,0);
        
        
        //cvSub(img_h,img_s,img_v);

        

       



        cvSetZero(edge);
cvSetImageROI(img_s,cvRect(roi->x,EDGE_Y_CUTOFF,roi->width,roi->height-EDGE_Y_CUTOFF+roi->y));
cvSetImageROI(img_v,cvRect(roi->x,EDGE_Y_CUTOFF,roi->width,roi->height-EDGE_Y_CUTOFF+roi->y));
        //cvSmooth(img_s,img_s,CV_GAUSSIAN,25,25,0);
        //cvSmooth(img_v,img_v,CV_GAUSSIAN,9,9,0);
cvSetImageROI(edge,cvRect(roi->x,EDGE_Y_CUTOFF,roi->width,roi->height-EDGE_Y_CUTOFF+roi->y));
cvSetImageROI(edge_search_region,cvRect(roi->x,EDGE_Y_CUTOFF,roi->width,roi->height-EDGE_Y_CUTOFF+roi->y));
 
        
//cvSetImageROI(img_s,cvRect(0,EDGE_Y_CUTOFF,PIC_WIDTH,PIC_HEIGHT-EDGE_Y_CUTOFF));
//cvSetImageROI(img_v,cvRect(0,EDGE_Y_CUTOFF,PIC_WIDTH,PIC_HEIGHT-EDGE_Y_CUTOFF));
//cvSetImageROI(edge,cvRect(0,EDGE_Y_CUTOFF,PIC_WIDTH,PIC_HEIGHT-EDGE_Y_CUTOFF));
//cvSetImageROI(edge_search_region,cvRect(0,EDGE_Y_CUTOFF,PIC_WIDTH,PIC_HEIGHT-EDGE_Y_CUTOFF));
        
        double minVal;
	double maxVal;

#ifdef CRYSTAL_LAKE
	cvSub(img_v,img_s,img_s);
	cvMinMaxLoc(img_s, &minVal,&maxVal,NULL);
        
	cvConvertScale(img_s, img_s, 255.0/(maxVal-minVal), -minVal/(maxVal-minVal) );
        cvSmooth(img_s,img_s,CV_GAUSSIAN,25,25,0);
        cvThreshold(img_s,img_s,60,255,CV_THRESH_BINARY_INV);
        
#endif   
        
#ifdef CRYSTAL_LAKE2
        //cvSub(img_v,img_s,img_s);
	cvMinMaxLoc(img_s, &minVal,&maxVal,NULL);
        
	cvConvertScale(img_s, img_s, 255.0/(maxVal-minVal), -minVal/(maxVal-minVal) );
        cvSmooth(img_s,img_s,CV_GAUSSIAN,15,15,0);
        cvThreshold(img_s,img_s,80,255,CV_THRESH_BINARY_INV);
        
        
        for (int x = 0; x < PIC_WIDTH; x++)
        {
            for (int y = 0; y < PIC_HEIGHT; y++)
            {
                if (y < -((Y_CHOP)*(x-X_CHOP)/(X_CHOP)))
                    (((uchar *)(img_s->imageData + y*img_s->widthStep))[x]) = 0;
                else if (y < ((Y_CHOP)*(x+X_CHOP-PIC_WIDTH)/(X_CHOP)))
                    (((uchar *)(img_s->imageData + y*img_s->widthStep))[x]) = 0;
            }
        }
        
        
        
#endif
        
        
#ifdef QUAD
        cvSub(img_v,img_s,img_s);
	cvMinMaxLoc(img_s, &minVal,&maxVal,NULL);
        
	cvConvertScale(img_s, img_s, 255.0/(maxVal-minVal), -minVal/(maxVal-minVal) );
        cvSmooth(img_s,img_s,CV_GAUSSIAN,15,15,0);
        cvThreshold(img_s,img_s,120,255,CV_THRESH_BINARY);
        
        
#endif
        
        
#ifdef MEADOWBROOK
        cvSub(img_v,img_s,img_s);
	cvMinMaxLoc(img_s, &minVal,&maxVal,NULL);
        
	cvConvertScale(img_s, img_s, 255.0/(maxVal-minVal), -minVal/(maxVal-minVal) );
        cvSmooth(img_s,img_s,CV_GAUSSIAN,15,15,0);
        cvThreshold(img_s,img_s,20,255,CV_THRESH_BINARY);
#endif
	//cvMinMaxLoc(img_v, &minVal,&maxVal,NULL);
	//cvConvertScale(img_v, img_v, 255.0/(maxVal-minVal), -minVal/(maxVal-minVal) );

	//cvMinMaxLoc(img_h, &minVal,&maxVal,NULL);
	//cvConvertScale(img_h, img_h, 255.0/(maxVal-minVal), -minVal/(maxVal-minVal) );

        cvCanny(img_s, edge, 0.000001, 30000.0, 7);
/*
        IplConvKernel* kernel = cvCreateStructuringElementEx(9, 9, 5, 5, CV_SHAPE_ELLIPSE, NULL);
        cvDilate(edge, edge_search_region, kernel, 1);
        kernel = cvCreateStructuringElementEx(13, 13, 7, 7, CV_SHAPE_ELLIPSE, NULL);
        cvErode(edge_search_region, edge_search_region, kernel, 1);

        kernel = cvCreateStructuringElementEx(15, 15, 8, 8, CV_SHAPE_ELLIPSE, NULL);
        cvDilate(edge_search_region, edge_search_region, kernel, 1);
*/
        /*IplConvKernel* kernel = cvCreateStructuringElementEx(15, 15, 8, 8, CV_SHAPE_ELLIPSE, NULL);
        cvErode(img_s, img_s, kernel, 1);
        kernel = cvCreateStructuringElementEx(15, 15, 8, 8, CV_SHAPE_ELLIPSE, NULL);
        cvDilate(img_s, img_s, kernel, 1);
        cvSub(img_s,img_v,img_s);*/
        
        
        
cvResetImageROI(img_s);
cvResetImageROI(img_v);
cvResetImageROI(edge);
cvResetImageROI(edge_search_region);


// find non-white blobs in thresholded image
//blobs = CBlobResult( img_s, NULL, 255 );
// exclude the ones smaller than param2 value
//blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 50 );

// get mean gray color of biggest blob
//CBlob biggestBlob;
//blobs.GetNthBlob( CBlobGetArea(), 0, biggestBlob );



#ifdef NO_SLAM
        sprintf(string,"ImageH %d",cam);
        cvShowImage(string,img_h);
        sprintf(string,"ImageS %d",cam);
        cvShowImage(string,img_s);
        sprintf(string,"ImageHS %d",cam);
        cvShowImage(string,img_v);
        cvWaitKey(10);
#endif
//sprintf(string,"ImageS %d",cam);
        //cvShowImage(string,img_s);

        cvCopy(edge,edge_search_region,NULL);
        if (cam == 0)
                createEdgeSequences(edge, edge_search_region, *roi, map_endpts);
        else
                createEdgeSequences(edge, edge_search_region, *roi);






        //cvCvtColor( edge, edge_clr, CV_GRAY2BGR );
		
        

#endif
        //sprintf(string,"Edge%d",cam);
        //cvShowImage(string,edge);
        //cvWaitKey(10);

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);


}


void FeatureDetector::plot_features(IplImage* img)
{

	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1,8);
	char feature_num[5];

	//Draw circles for corners back on original image
	for( int n = 0; n < num_features; n++)
	{
		//sprintf(feature_num,"%d", n);
		int radius = 5;
		cvCircle(img,
			cvPoint((int)(feature_points[n].x + 0.5f),(int)(feature_points[n].y + 0.5f)),radius,CV_RGB(0,0,0));
		//cvPutText(img,feature_num,cvPoint((int)(feature_points[n].x - 5),(int)(feature_points[n].y - 5)),&font,CV_RGB(0,0,0));

	}

}

	
std::vector<CvPoint> ** FeatureDetector::return_features()
{
	return &(features[0]);
}
	
float FeatureDetector::getTime()
{
	return elapsedTime;
}


void FeatureDetector::resetTime()
{
    elapsedTime = 0.0;
}



/*
void FeatureDetector::edge_following(IplImage *img)
{

    IplImage * edge = cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
    cvCopy(img,edge);
    cvSetZero(img);


    featuresLeftCurve.resize(0);
    featuresRightCurve.resize(0);
    features[0] = &(featuresLeftCurve);
    features[1] = &(featuresRightCurve);


        current_left_x = prev_left_x;
        current_left_y = prev_left_y;
        current_right_x = prev_right_x;
        current_right_y = prev_right_y;
        //Need to allow for a range, and favour those below than above
        if (current_left_x == 0)
            current_left_y+=20;
        else
            current_left_x+=20;

        if (current_right_x == PIC_WIDTH-1)
            current_right_y+=20;
        else
            current_right_x-=20;

        current_left_x = BOX(current_left_x,0,PIC_WIDTH-1);
        current_left_y = BOX(current_left_y,0,PIC_HEIGHT-1);
        current_right_x = BOX(current_right_x,0,PIC_WIDTH-1);
        current_right_y = BOX(current_right_y,0,PIC_HEIGHT-1);



        //STEP 1: Move outwards until we find an edge pixel to start with
        bool move_up = false;
        if(!current_left_x)
                    move_up = true;
        //cout << "LEFT FIRST\n";
        while(1)
        {
            double s = cvGet2D(edge,current_left_y,current_left_x).val[0];
            double s2 = cvGet2D(edge,current_left_y-1,current_left_x).val[0];
            if ( s || s2)
            {
                cvSet2D(edge,current_left_y,current_left_x,cvScalar(255));
                break;
            }
            else
            {
                if(move_up)
                    current_left_y--;
                else
                    current_left_x--;

                if(!current_left_x)
                    move_up = true;
            }

            cout << "{ " << current_left_x << ", " << current_left_y << " }" << endl;
        }
        move_up = false;
        if(current_right_x == PIC_WIDTH-1)
                    move_up = true;
        //cout << "RIGHT FIRST\n";
        
        while(1)
        {
            double s = cvGet2D(edge,current_right_y,current_right_x).val[0];
            double s2 = cvGet2D(edge,current_right_y-1,current_right_x).val[0];
            if ( s || s2)
            {
                cvSet2D(edge,current_right_y,current_right_x,cvScalar(255));
                break;
            }
            else
            {
                if(move_up)
                    current_right_y--;
                else
                    current_right_x++;

                if(current_right_x == PIC_WIDTH-1)
                    move_up = true;
            }

            cout << "{ " << current_right_x << ", " << current_right_y << " }" << endl;
        }

        move_up = false;


        //Store the boundary edge starting pts, so we know to start here next time
        //prev_left_x = current_left_x;
        //prev_left_y = current_left_y;
        //prev_right_x = current_right_x;
        //prev_right_y = current_right_y;


        //Store last 5 edge points, so we can estimate the edge direction
        std::vector<CvPoint> edge_history;
        CvPoint current = cvPoint(current_left_x-1,current_left_y+1);
        int edge_bufsize = 5;
        for (int i = 0; i < edge_bufsize; i++)
            edge_history.push_back(current);
        double edge_direction = 0.0;

        
        //STEP 2: START EDGE FOLLOWING, maintain a direction too
        //LEFT
        //cout << "LEFT SECOND\n";
        while(1)
        {
            //Add current pixel to feature vector
            current = cvPoint(current_left_x,current_left_y);
            featuresLeftCurve.push_back(current);
            edge_history.pop_front();
            edge_history.push_back(current);
            edge_direction = atan2(current.y-edge_history[0].y,current.x-edge_history[0].x);
            cvSet2D(img,current_left_y,current_left_x,cvScalar(255));

            if (current_left_x >= 159 || current_left_y <= EDGE_Y_CUTOFF)
                break;
            
            if (cvGet2D(edge,current_left_y,current_left_x+1).val[0])
            {
                current_left_x++;
            }
            else if (cvGet2D(edge,current_left_y-1,current_left_x+1).val[0])
            {
                current_left_y--;
                current_left_x++;
            }
            else if (cvGet2D(edge,current_left_y-1,current_left_x).val[0])
            {
                current_left_y--;
            }
            else if (current_left_x > 0 && cvGet2D(edge,current_left_y-1,current_left_x-1).val[0])
            {
                current_left_x--;
                current_left_y--;
            }
            else if (cvGet2D(edge,current_left_y-2,current_left_x).val[0])
            {
                current_left_y-=2;
            }
            else if (cvGet2D(edge,current_left_y-2,current_left_x+1).val[0])
            {
                current_left_y-=2;
                current_left_x++;
            }
            else if (cvGet2D(edge,current_left_y-2,current_left_x+2).val[0])
            {
                current_left_y-=2;
                current_left_x+=2;
            }
            else if (cvGet2D(edge,current_left_y-1,current_left_x+2).val[0])
            {
                current_left_y--;
                current_left_x+=2;
            }
            else if (cvGet2D(edge,current_left_y,current_left_x+2).val[0])
            {
                current_left_x+=2;
            }
            else
                break;


            cout << "{ " << current_left_x << ", " << current_left_y << " }" << endl;
        }
        //RIGHT
        //cout << "RIGHT SECOND\n";
        while(1)
        {
            current = cvPoint(current_right_x,current_right_y);
            featuresRightCurve.push_back(current);
            cvSet2D(img,current_right_y,current_right_x,cvScalar(255));

            if (current_right_x <= 161 || current_right_y <= EDGE_Y_CUTOFF)
                break;

            
            if (cvGet2D(edge,current_right_y,current_right_x-1).val[0])
            {
                current_right_x--;
            }
            else if (cvGet2D(edge,current_right_y-1,current_right_x-1).val[0])
            {
                current_right_y--;
                current_right_x--;
            }
            else if (cvGet2D(edge,current_right_y-1,current_right_x).val[0])
            {
                current_right_y--;
            }
            else if (current_right_x < PIC_WIDTH-1 && cvGet2D(edge,current_right_y-1,current_right_x+1).val[0])
            {
                current_right_x++;
                current_right_y--;
            }
            else if (cvGet2D(edge,current_right_y-2,current_right_x).val[0])
            {
                current_right_y-=2;
            }
            else if (cvGet2D(edge,current_right_y-2,current_right_x-1).val[0])
            {
                current_right_y-=2;
                current_right_x--;
            }
            else if (cvGet2D(edge,current_right_y-2,current_right_x-2).val[0])
            {
                current_right_y-=2;
                current_right_x-=2;
            }
            else if (cvGet2D(edge,current_right_y-1,current_right_x-2).val[0])
            {
                current_right_y--;
                current_right_x-=2;
            }
            else if (cvGet2D(edge,current_right_y,current_right_x-2).val[0])
            {
                current_right_x-=2;
            }
            else
                break;

            cout << "{ " << current_right_x << ", " << current_right_y << " }" << endl;

        }






        cvReleaseImage(&edge);
}
*/

/*
void FeatureDetector::createEdgeSequences(IplImage * edge, IplImage * edge_search_region)
{
    featuresLeftCurve.resize(0);
    featuresRightCurve.resize(0);
    features[0] = &(featuresLeftCurve);
    features[1] = &(featuresRightCurve);

    //Only keep edge points within distance of the search region
        for(int y = PIC_HEIGHT-1; y >= EDGE_Y_CUTOFF; y--)
        {
                for(int x = 0; x < PIC_WIDTH/2; x++)
                {
                        //printf("%d, %d\n",x,y);
                        //printf("Size: %d, %d",seg_img->height,seg_img->width);

                        //If it is an edge point
                        if (((uchar *)(edge_search_region->imageData + y*edge_search_region->widthStep))[x])
                        {
                                if (((uchar *)(edge->imageData + y*edge->widthStep))[x])
                                //If it's in the segmented region then add to feature vector
                                {
                                    featuresLeftCurve.push_back(cvPoint(x,y));
                                    //cout << "{ " << x << ", " << y << " }" << endl;
                                }
                        }
                        else
                            (((uchar *)(edge->imageData + y*edge->widthStep))[x]) = 0;
                }
        }

        for(int y = PIC_HEIGHT-1; y >= EDGE_Y_CUTOFF; y--)
        {
                for(int x = PIC_WIDTH-1; x >= PIC_WIDTH/2; x--)
                {
                        //printf("%d, %d\n",x,y);
                        //printf("Size: %d, %d",seg_img->height,seg_img->width);

                        //If it is an edge point
                        if (((uchar *)(edge_search_region->imageData + y*edge_search_region->widthStep))[x])
                        {
                                if (((uchar *)(edge->imageData + y*edge->widthStep))[x])
                                //If it's in the segmented region then add to feature vector
                                {
                                    featuresRightCurve.push_back(cvPoint(x,y));
                                    //cout << "{ " << x << ", " << y << " }" << endl;
                                }
                        }
                        else
                            (((uchar *)(edge->imageData + y*edge->widthStep))[x]) = 0;
                }
        }



}
*/


void FeatureDetector::findFloodFillSeed(IplImage * edge_search_region, CvPoint * seed)
{
    int in_a_row = 0;

    for(int y = 230; y >= EDGE_Y_CUTOFF; y--)
    {
            for(int x = 0; x < PIC_WIDTH/2; x++)
            {
                    //If it is an edge point
                    if (((uchar *)(edge_search_region->imageData + y*edge_search_region->widthStep))[x])
                    {
                        in_a_row++;
                        if(in_a_row > 3)
                        {
                            seed[0].x = x;
                            seed[0].y = y;
                            break;
                        }
                    }
                    else
                        in_a_row = 0;
            }
            in_a_row = 0;
    }

    in_a_row = 0;
    for(int y = 230; y >= EDGE_Y_CUTOFF; y--)
    {
            for(int x = PIC_WIDTH-1; x >= PIC_WIDTH/2; x--)
            {
                    //If it is an edge point
                    if (((uchar *)(edge_search_region->imageData + y*edge_search_region->widthStep))[x])
                    {
                        in_a_row++;
                        if(in_a_row > 3)
                        {
                            seed[1].x = x;
                            seed[1].y = y;
                            break;
                        }
                    }
                    else
                        in_a_row = 0;
            }
            in_a_row = 0;
    }
    
}



/*

        IplImage * df_dx = cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,1);
        IplImage * df_dy = cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,1);
        IplImage * mag = cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,1);
        IplImage * direction = cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,1);

cvSetZero( mag );
cvSetZero( direction );

        cvSobel( img, df_dx, 1, 0, 3);
        cvSobel( img, df_dy, 0, 1, 3);


	double minVal;
	double maxVal;

	cvMinMaxLoc(df_dx, &minVal,&maxVal,NULL);
	cvConvertScale(df_dx, df_dx, 2.0/(maxVal-minVal), -2.0*minVal/(maxVal-minVal) -1.0 );
cvSquareAcc( df_dx, mag );
	cvMinMaxLoc(df_dy, &minVal,&maxVal,NULL);
	cvConvertScale(df_dy, df_dy, 2.0/(maxVal-minVal), -2.0*minVal/(maxVal-minVal) -1.0 );
cvSquareAcc( df_dy, mag );

cvbSqrt( (float*)(mag->imageData), (float*)(mag->imageData),
mag->imageSize/sizeof(float));


        cvSmooth(mag, mag, CV_GAUSSIAN, 15, 15, 0, 0);
        cvConvertImage(mag,edge);
        cvCanny(edge, edge, 0.001, 50.0, 3.0);

for (int x = 0; x < PIC_WIDTH; x++)
{
    for (int y = 0; y < PIC_HEIGHT; y++)
    {
        //float magval = pow( pow( ((uchar *)(df_dx->imageData + y*df_dx->widthStep))[x], 2.0 ) + pow( ((uchar *)(df_dy->imageData + y*df_dy->widthStep))[x], 2.0 ) , 0.5 );
        //((uchar *)(mag->imageData + y*mag->widthStep))[x] = magval;

        float dirval = atan2( ((float *)(df_dy->imageData + y*df_dy->widthStep))[x], ((float *)(df_dx->imageData + y*df_dx->widthStep))[x] );
        ((float *)(direction->imageData + y*direction->widthStep))[x] = dirval;


        if (!( (dirval > PI/4 && dirval < 3*PI/4) || (dirval < -PI/4 && dirval > -3*PI/4) ) )
            ((float *)(mag->imageData + y*mag->widthStep))[x] = 0.0;

    }
}*/








void FeatureDetector::createEdgeSequences(IplImage * edge, IplImage * edge_search_region, CvRect roi)
{
    featuresLeftCurve.resize(0);
    featuresRightCurve.resize(0);
    features[0] = &(featuresLeftCurve);
    features[1] = &(featuresRightCurve);
    cvSetZero(edge);

    //Only keep edge points within distance of the search region
    float avg_x = 0;
    float num_pts = 0;
        for(int y = PIC_HEIGHT-1; y >= EDGE_Y_CUTOFF; y--)
        {
                for(int x = 0; x < PIC_WIDTH/2; x++)
                {

                        //If it is an edge point
                        if (((uchar *)(edge_search_region->imageData + y*edge_search_region->widthStep))[x])
                        {
                                //if (((uchar *)(edge->imageData + y*edge->widthStep))[x])
                                //If it's in the segmented region then add to feature vector
                                {
                                    //featuresLeftCurve.push_back(cvPoint(x,y));
                                    //cout << "{ " << x << ", " << y << " }" << endl;
                                    avg_x += x;
                                    num_pts++;
                                }
                        }
                        (((uchar *)(edge->imageData + y*edge->widthStep))[x]) = 0;


                }
                if(num_pts)
                {
                    avg_x /= num_pts;
                    featuresLeftCurve.push_back(cvPoint(avg_x,y));
                    (((uchar *)(edge->imageData + y*edge->widthStep))[(int)(avg_x)]) = 255;
                }
                num_pts = 0;
                avg_x = 0;
        }

        for(int y = PIC_HEIGHT-1; y >= EDGE_Y_CUTOFF; y--)
        {
                for(int x = PIC_WIDTH-1; x >= PIC_WIDTH/2; x--)
                {

                        //If it is an edge point
                        if (((uchar *)(edge_search_region->imageData + y*edge_search_region->widthStep))[x])
                        {
                                //if (((uchar *)(edge->imageData + y*edge->widthStep))[x])
                                //If it's in the segmented region then add to feature vector
                                {
                                    //featuresRightCurve.push_back(cvPoint(x,y));
                                    //cout << "{ " << x << ", " << y << " }" << endl;
                                    avg_x += x;
                                    num_pts++;
                                }
                        }
                        (((uchar *)(edge->imageData + y*edge->widthStep))[x]) = 0;


                }
                if(num_pts)
                {
                    avg_x /= num_pts;
                    featuresRightCurve.push_back(cvPoint(avg_x,y));
                    (((uchar *)(edge->imageData + y*edge->widthStep))[(int)(avg_x)]) = 255;
                }
                num_pts = 0;
                avg_x = 0;
        }



}
void FeatureDetector::createEdgeSequences(IplImage * edge, IplImage * edge_search_region, CvRect roi, CvPoint2D32f * map_endpts)
{
    featuresLeftCurve.resize(0);
    featuresRightCurve.resize(0);
    features[0] = &(featuresLeftCurve);
    features[1] = &(featuresRightCurve);
    cvSetZero(edge);
    
    float dist_to_tracked_pt[] = {1000.0, 1000.0};
    CvPoint new_tracked_pts[] = {cvPoint(0,0),cvPoint(0,0)};

    //Only keep edge points within distance of the search region
    float avg_x = 0;
    float num_pts = 0;
    
        for(int y = PIC_HEIGHT-1; y >= EDGE_Y_CUTOFF; y--)
        {
                for(int x = 0; x < PIC_WIDTH/2; x++)
                {

                        //If it is an edge point
                        if (((uchar *)(edge_search_region->imageData + y*edge_search_region->widthStep))[x])
                        {
                                //if (((uchar *)(edge->imageData + y*edge->widthStep))[x])
                                //If it's in the segmented region then add to feature vector
                                {
                                    //featuresLeftCurve.push_back(cvPoint(x,y));
                                    //cout << "{ " << x << ", " << y << " }" << endl;
                                    avg_x += x;
                                    num_pts++;
                                }
                        }
                        (((uchar *)(edge->imageData + y*edge->widthStep))[x]) = 0;


                }
                if(num_pts)
                {
                    avg_x /= num_pts;
                    CvPoint edgept = cvPoint(avg_x,y);
                    featuresLeftCurve.push_back(edgept);
                    float current_dist = CVPOINT_DIST(edgept,map_endpts[0]);
                    if (current_dist < dist_to_tracked_pt[0])
                    {
                        dist_to_tracked_pt[0] = current_dist;
                        new_tracked_pts[0] = edgept;
                    }
                    (((uchar *)(edge->imageData + y*edge->widthStep))[(int)(avg_x)]) = 255;
                }
                num_pts = 0;
                avg_x = 0;
        }

        for(int y = PIC_HEIGHT-1; y >= EDGE_Y_CUTOFF; y--)
        {
                for(int x = PIC_WIDTH-1; x >= PIC_WIDTH/2; x--)
                {

                        //If it is an edge point
                        if (((uchar *)(edge_search_region->imageData + y*edge_search_region->widthStep))[x])
                        {
                                //if (((uchar *)(edge->imageData + y*edge->widthStep))[x])
                                //If it's in the segmented region then add to feature vector
                                {
                                    //featuresRightCurve.push_back(cvPoint(x,y));
                                    //cout << "{ " << x << ", " << y << " }" << endl;
                                    avg_x += x;
                                    num_pts++;
                                }
                        }
                        (((uchar *)(edge->imageData + y*edge->widthStep))[x]) = 0;


                }
                if(num_pts)
                {
                    avg_x /= num_pts;
                    CvPoint edgept = cvPoint(avg_x,y);
                    featuresRightCurve.push_back(edgept);
                    float current_dist = CVPOINT_DIST(edgept,map_endpts[1]);
                    if (current_dist < dist_to_tracked_pt[1])
                    {
                        dist_to_tracked_pt[1] = current_dist;
                        new_tracked_pts[1] = edgept;
                    }
                    (((uchar *)(edge->imageData + y*edge->widthStep))[(int)(avg_x)]) = 255;
                }
                num_pts = 0;
                avg_x = 0;
        }
        
    map_endpts[0] = cvPoint2D32f(new_tracked_pts[0].x,new_tracked_pts[0].y);
    map_endpts[1] = cvPoint2D32f(new_tracked_pts[1].x,new_tracked_pts[1].y);
}
