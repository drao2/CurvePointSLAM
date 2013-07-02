#include "featureDetector.h"
#include "capture.h"
//#include "cvblobs/BlobResult.h"
//#include "cvblobs/BlobProperties.h"

using namespace std;



FeatureDetector::FeatureDetector()
{
	gettimeofday(&start, NULL);
        
        //Temporary images for edge detection
	size = cvSize(PIC_WIDTH,PIC_HEIGHT);
	eig_image = cvCreateImage(size,IPL_DEPTH_32F,1);
	edge = cvCreateImage(size,IPL_DEPTH_8U,1);
        img = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);
        edge_search_region = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);
        last_edge_search_region = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);

        img_hsv = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,3);
        img_h = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);
        img_s = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);
        img_v = cvCreateImage(cvSize(PIC_WIDTH,PIC_HEIGHT),IPL_DEPTH_8U,1);

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

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
}

//Find edge features using segmentation and edge detection
//Tweak this function for different environments, a few are already preset using #ifdef
void FeatureDetector::find_features(IplImage* img, IplImage* seg_img, int cam, CvRect * roi, CvPoint2D32f * map_endpts)
{
	gettimeofday(&start, NULL);

        //Convert to HSV
        cvCvtColor(img,img_hsv,CV_RGB2HSV);

        //Extract channels from hsv image
        char string[20];
        cvSetImageCOI(img_hsv,1);
        cvCopy(img_hsv,img_h,NULL);
        cvSetImageCOI(img_hsv,2);
        cvCopy(img_hsv,img_s,NULL);
        cvSetImageCOI(img_hsv,3);
        cvCopy(img_hsv,img_v,NULL);
        cvSetImageCOI(img_hsv,0);

        //Set ROI based on distortion coeffs, etc
        cvSetZero(edge);
        cvSetImageROI(img_s,cvRect(roi->x,EDGE_Y_CUTOFF,roi->width,roi->height-EDGE_Y_CUTOFF+roi->y));
        cvSetImageROI(img_v,cvRect(roi->x,EDGE_Y_CUTOFF,roi->width,roi->height-EDGE_Y_CUTOFF+roi->y));
        //cvSmooth(img_s,img_s,CV_GAUSSIAN,25,25,0);
        //cvSmooth(img_v,img_v,CV_GAUSSIAN,9,9,0);
        cvSetImageROI(edge,cvRect(roi->x,EDGE_Y_CUTOFF,roi->width,roi->height-EDGE_Y_CUTOFF+roi->y));
        cvSetImageROI(edge_search_region,cvRect(roi->x,EDGE_Y_CUTOFF,roi->width,roi->height-EDGE_Y_CUTOFF+roi->y));
        
        double minVal;
	double maxVal;

//BASED ON THE DATASET / ENVIRONMENT, THE SEGMENTATION IS SLIGHTLY DIFFERENT!
// EG. FOR SOME, COLOUR IS VERY STRONG, FOR OTHERS, NOT SO MUCH
        
#ifdef CRYSTAL_LAKE
        //cvSetZero(img_v);
        //cvAddS(img_v,cvScalar(255),img_v,NULL);
	//cvSub(img_v,img_s,img_s);
	cvMinMaxLoc(img_s, &minVal,&maxVal,NULL);
        
	cvConvertScale(img_s, img_s, 255.0/(maxVal-minVal), -minVal/(maxVal-minVal) );
        cvSmooth(img_s,img_s,CV_GAUSSIAN,25,25,0);
        cvThreshold(img_s,img_s,60,255,CV_THRESH_BINARY);
        
        //Cut out top left and top right of image diagonally (We're assuming path
        //will only appear in certain spots)
        /*for (int x = 0; x < PIC_WIDTH; x++)
        {
            for (int y = 0; y < PIC_HEIGHT; y++)
            {
                if (y < -((Y_CHOP)*(x-X_CHOP)/(X_CHOP)))
                    (((uchar *)(img_s->imageData + y*img_s->widthStep))[x]) = 0;
                else if (y < ((Y_CHOP)*(x+X_CHOP-PIC_WIDTH)/(X_CHOP)))
                    (((uchar *)(img_s->imageData + y*img_s->widthStep))[x]) = 0;
            }
        }*/
        
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

        //Apply edge detection
        cvCanny(img_s, edge, 0.000001, 30000.0, 7);
        
//Additional morphological operations if necessary (CURRENTLY NOT USED)
/*
        IplConvKernel* kernel = cvCreateStructuringElementEx(9, 9, 5, 5, CV_SHAPE_ELLIPSE, NULL);
        cvDilate(edge, edge_search_region, kernel, 1);
        kernel = cvCreateStructuringElementEx(13, 13, 7, 7, CV_SHAPE_ELLIPSE, NULL);
        cvErode(edge_search_region, edge_search_region, kernel, 1);

        kernel = cvCreateStructuringElementEx(15, 15, 8, 8, CV_SHAPE_ELLIPSE, NULL);
        cvDilate(edge_search_region, edge_search_region, kernel, 1);

        IplConvKernel* kernel = cvCreateStructuringElementEx(15, 15, 8, 8, CV_SHAPE_ELLIPSE, NULL);
        cvErode(img_s, img_s, kernel, 1);
        kernel = cvCreateStructuringElementEx(15, 15, 8, 8, CV_SHAPE_ELLIPSE, NULL);
        cvDilate(img_s, img_s, kernel, 1);
        cvSub(img_s,img_v,img_s);*/
        
//Garbage collection
cvResetImageROI(img_s);
cvResetImageROI(img_v);
cvResetImageROI(edge);
cvResetImageROI(edge_search_region);

//Can define NO_SLAM if we want to just look at the segmentation/ edge
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

        //Get edge image / vectors containing sequences of edge points
        cvCopy(edge,edge_search_region,NULL);
        if (cam == 0)
                createEdgeSequences(edge, edge_search_region, *roi, map_endpts);
        else
                createEdgeSequences(edge, edge_search_region, *roi);

        //sprintf(string,"Edge%d",cam);
        //cvShowImage(string,edge);
        //cvWaitKey(10);

	gettimeofday(&stop, NULL);
	elapsedTime += (stop.tv_sec*1000.0 + stop.tv_usec/1000.0) -
		(start.tv_sec*1000.0 + start.tv_usec/1000.0);
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
